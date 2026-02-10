#!/usr/bin/env python3
# Plot XY path from NPZ files (supports _partXXXX.npz sequences).

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List

import numpy as np


def _resolve_npz_files(path: Path) -> List[Path]:
    if path.is_dir():
        files = sorted(path.glob("*.npz"))
        return files

    if path.suffix != ".npz":
        path = path.with_suffix(".npz")

    if path.exists():
        return [path]

    # Try part files: <base>_part*.npz
    dir_path = path.parent if path.parent.as_posix() else Path(".")
    stem = path.stem
    prefix = f"{stem}_part"
    files = sorted([p for p in dir_path.glob("*.npz") if p.stem.startswith(prefix)])
    return files


def load_xy(files: List[Path]) -> np.ndarray:
    xs = []
    ys = []
    for f in files:
        data = np.load(str(f))
        if "x" not in data or "y" not in data:
            continue
        xs.append(data["x"].astype(np.float32, copy=False))
        ys.append(data["y"].astype(np.float32, copy=False))
    if not xs:
        return np.empty((0, 2), dtype=np.float32)
    x = np.concatenate(xs)
    y = np.concatenate(ys)
    return np.stack([x, y], axis=1)


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot XY trajectory from NPZ files.")
    parser.add_argument("--npz", required=True, help="NPZ file path, base path, or directory")
    parser.add_argument("--out", default="xy_path.png", help="Output image path (png)")
    parser.add_argument("--dpi", type=int, default=150, help="Output image DPI")
    parser.add_argument("--stride", type=int, default=1, help="Plot every Nth point to speed up")
    args = parser.parse_args()

    npz_path = Path(args.npz).expanduser().resolve()
    files = _resolve_npz_files(npz_path)
    if not files:
        raise SystemExit(f"[error] no npz files found for: {npz_path}")

    xy = load_xy(files)
    if xy.size == 0:
        raise SystemExit("[error] no x/y data found in npz files")

    if args.stride > 1:
        xy = xy[:: args.stride]

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8), dpi=args.dpi)
    ax.plot(xy[:, 0], xy[:, 1], linewidth=0.6, color="#2b2b2b")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("XY Path")
    ax.grid(True, linewidth=0.3, alpha=0.5)

    out_path = Path(args.out).expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(out_path), bbox_inches="tight")
    print(f"[info] saved: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
