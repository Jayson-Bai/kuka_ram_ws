"""Lossless gzip chunks in memory; files are written only on explicit export."""
import gzip
import json


class DiagnosticLog:
    """Keep the entire UI session, compacting records without time sampling."""

    def __init__(self, chunk_bytes=65536):
        self._chunk_bytes = chunk_bytes
        self._pending = bytearray()
        self._chunks = []
        self.record_count = 0
        self.raw_bytes = 0
        self.compressed_bytes = 0

    def __bool__(self):
        return self.record_count > 0

    def append(self, record):
        # Local display time is derivable from time_epoch. UART direction and
        # payload are derivable from line. Keep the raw line, including warnings.
        compact = dict(record)
        compact.pop('time', None)
        if compact.get('source') == 'uart' and compact.get('kind') == 'raw':
            detail = dict(compact['detail'])
            if 'line' in detail:
                detail.pop('payload', None)
                detail.pop('direction', None)
            compact['detail'] = detail
        payload = (json.dumps(compact, ensure_ascii=False, separators=(',', ':'))
                   + '\n').encode('utf-8')
        self._pending.extend(payload)
        self.record_count += 1
        self.raw_bytes += len(payload)
        if len(self._pending) >= self._chunk_bytes:
            self._seal()

    def _seal(self):
        if self._pending:
            chunk = gzip.compress(bytes(self._pending), compresslevel=1, mtime=0)
            self._chunks.append(chunk)
            self.compressed_bytes += len(chunk)
            self._pending.clear()

    def export(self, path):
        """Concatenated gzip members are readable as one JSONL gzip stream."""
        self._seal()
        with open(path, 'xb') as stream:
            for chunk in self._chunks:
                stream.write(chunk)
        return self.compressed_bytes
