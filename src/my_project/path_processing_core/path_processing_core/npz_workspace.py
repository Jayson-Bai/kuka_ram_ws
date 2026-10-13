"""Disk-backed arrays for large NPZs; no trajectory or sampling semantics."""
from __future__ import annotations

from contextlib import AbstractContextManager
from contextvars import ContextVar
from itertools import islice
from pathlib import Path
import mmap
import shutil
import tempfile
import zipfile

import numpy as np


IN_MEMORY_BYTES = 256 * 1024 ** 2
COPY_BYTES = 8 * 1024 ** 2
REBUILD_BYTES = 256 * 1024 ** 2
_ACTIVE_WORKSPACE = ContextVar("npz_workspace", default=None)


def materialize_samples(samples):
    if _ACTIVE_WORKSPACE.get() is None:
        return list(samples)
    # Sampling uses small dataclass objects, before row dictionaries exist.
    limit = max(1, REBUILD_BYTES // 512)
    result = list(islice(samples, limit + 1))
    if len(result) > limit:
        raise ValueError(
            "local injection sampling buffer exceeds the safe in-memory row budget; "
            "use Core full export for this parameter magnitude"
        )
    return result


def concatenated_dtype(columns):
    # Match the installed NumPy, including its single-input byte-order rules.
    # Empty prototypes avoid allocating or reading the actual column payload.
    return np.concatenate([np.empty(0, dtype=column.dtype) for column in columns]).dtype


def uncompressed_bytes(files):
    total = 0
    for path in files:
        with zipfile.ZipFile(path) as archive:
            total += sum(item.file_size for item in archive.infolist())
    return total


class NpzWorkspace(AbstractContextManager):
    """Own all temporary mappings and remove them on success or failure."""

    def __init__(self):
        self.directory = tempfile.TemporaryDirectory(prefix="core-npz-")
        self.root = Path(self.directory.name)
        self.mappings = {}
        self.counter = 0

    def __enter__(self):
        self.token = _ACTIVE_WORKSPACE.set(self)
        return self

    def __exit__(self, *exc):
        try:
            for array in list(self.mappings.values()):
                self.release(array)
            self.directory.cleanup()
        finally:
            _ACTIVE_WORKSPACE.reset(self.token)

    def check_space(self, required):
        # Leave room for filesystem metadata and unrelated small writes.
        if int(required) + 64 * 1024 ** 2 > shutil.disk_usage(self.root).free:
            raise ValueError(
                "local injection temporary disk budget exceeded: "
                f"requires {int(required):,} additional bytes in {self.root.parent}"
            )

    def _path(self):
        self.counter += 1
        return self.root / f"array-{self.counter}.npy"

    def allocate(self, shape, dtype):
        if not np.prod(shape):
            return np.empty(shape, dtype=dtype)
        self.check_space(int(np.prod(shape)) * np.dtype(dtype).itemsize)
        result = np.lib.format.open_memmap(
            self._path(), mode="w+", dtype=dtype, shape=shape,
        )
        self.mappings[id(result)] = result
        return result

    def read(self, path):
        result = {}
        with np.load(path, allow_pickle=False) as index, zipfile.ZipFile(path) as archive:
            for key in index.files:
                member = key + ".npy"
                self.check_space(archive.getinfo(member).file_size)
                target = self._path()
                with archive.open(member) as source, target.open("wb") as output:
                    shutil.copyfileobj(source, output, COPY_BYTES)
                # NumPy validates the NPY header and rejects object arrays.
                array = np.load(target, mmap_mode="r+", allow_pickle=False)
                self.mappings[id(array)] = array
                result[key] = array
        return result

    @staticmethod
    def copy(destination, source):
        if destination.ndim == 0:
            destination[...] = source
            return
        step = max(1, COPY_BYTES // max(destination.dtype.itemsize, 1))
        for start in range(0, len(source), step):
            destination[start:start + step] = source[start:start + step]

    def concatenate(self, columns):
        dtype = concatenated_dtype(columns)
        result = self.allocate((sum(len(column) for column in columns),), dtype)
        offset = 0
        for column in columns:
            self.copy(result[offset:offset + len(column)], column)
            self.evict(column)
            offset += len(column)
        self.evict(result)
        return result

    @staticmethod
    def evict(array):
        mapping = getattr(array, "_mmap", None)
        if mapping is not None and hasattr(mapping, "madvise") and hasattr(mmap, "MADV_DONTNEED"):
            # Shared file mappings keep their bytes in the backing file/page
            # cache. Drop this process's resident pages after sequential I/O.
            mapping.madvise(mmap.MADV_DONTNEED)

    def release(self, array):
        owned = self.mappings.pop(id(array), None)
        if owned is not None:
            filename = Path(owned.filename)
            owned._mmap.close()
            filename.unlink(missing_ok=True)


class DiskArrays(dict):
    """Keep normal ndarray indexing while replacing full-column copies on disk."""

    def __init__(self, workspace):
        super().__init__()
        self.workspace = workspace
        self.buffers = {}

    def put(self, key, value):
        previous = self.buffers.get(key)
        self.buffers[key] = value
        dict.__setitem__(self, key, value)
        if previous is not value:
            self.workspace.release(previous)

    def __setitem__(self, key, value):
        previous = self.get(key)
        if previous is value:
            return
        if previous is not None and previous.shape == value.shape and previous.dtype == value.dtype:
            self.workspace.copy(previous, value)
        else:
            target = self.workspace.allocate(value.shape, value.dtype)
            self.workspace.copy(target, value)
            self.put(key, target)

    def check_size(self, rows):
        dtype = self["seq"].dtype
        if rows and rows - 1 > np.iinfo(dtype).max:
            raise ValueError("local injection row count exceeds NPZ seq dtype capacity")
        # Conservatively reserve a full output, replacement workspace and ZIP.
        row_bytes = sum(value.dtype.itemsize for value in self.values())
        self.workspace.check_space(int(rows) * row_bytes * 3)

    def check_row_objects(self, count):
        # Rebuilders intentionally retain their existing Python row objects.
        # Limit a single synthetic region separately from the whole NPZ.
        row_bytes = 256 + sum(96 + value.dtype.itemsize for value in self.values())
        if int(count) * row_bytes > REBUILD_BYTES:
            raise ValueError(
                "local injection synthetic block exceeds the safe in-memory row budget; "
                "use Core full export for this parameter magnitude"
            )

    def replace(self, start, end, replacement):
        original_size = len(self["seq"])
        replacement_size = len(replacement["seq"])
        rows = original_size - (end - start) + replacement_size
        self.check_size(rows)
        for key in list(self):
            buffer = self.buffers[key]
            delta = rows - original_size
            dtype = concatenated_dtype((buffer, replacement[key], buffer))
            if rows > len(buffer) or dtype != buffer.dtype:
                # Reuse modest spare capacity across successive block edits.
                # Only the logical prefix is ever passed to the NPZ writer.
                capacity = max(rows, original_size + max(100000, original_size // 20))
                target = self.workspace.allocate((capacity,), dtype)
                self.workspace.copy(target[:start], buffer[:start])
                self.workspace.copy(target[start:start + replacement_size], replacement[key])
                self.workspace.copy(target[start + replacement_size:rows], buffer[end:original_size])
                self.put(key, target)
                buffer = target
            else:
                step = max(1, COPY_BYTES // max(buffer.dtype.itemsize, 1))
                if delta > 0:
                    # Move the tail backward for overlapping right shifts.
                    stop = original_size
                    while stop > end:
                        begin = max(end, stop - step)
                        buffer[begin + delta:stop + delta] = buffer[begin:stop].copy()
                        stop = begin
                elif delta < 0:
                    for begin in range(end, original_size, step):
                        stop = min(original_size, begin + step)
                        buffer[begin + delta:stop + delta] = buffer[begin:stop].copy()
                buffer[start:start + replacement_size] = replacement[key]
            dict.__setitem__(self, key, buffer[:rows])
            self.workspace.evict(buffer)
