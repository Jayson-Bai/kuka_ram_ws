#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线流水线 CLI：读取 GCode -> 解析 -> B 样条拟合/采样 -> 导出 npz 分片。
"""

import argparse
import os
import sys

from .gcode_parser import (
    load_gcode_lines,
    parse_gcode_lines,
    _select_default_gcode_file,
    default_data_root,
)
from .npz_exporter import export_npz


def _default_output_path(gcode_path: str, output_dir: str) -> str:
    base = os.path.splitext(os.path.basename(gcode_path))[0]
    return os.path.join(output_dir, base + ".npz")


def main(argv=None):
    parser = argparse.ArgumentParser(description="GCode -> NPZ 离线导出（分片）")
    parser.add_argument("--gcode", type=str, help="GCode 文件路径，缺省则从 input_gcode_dir 挑首个 .gcode")
    parser.add_argument("--data-root", type=str, default=default_data_root(), help="默认数据根目录")
    parser.add_argument("--input-gcode-dir", type=str, default="", help="GCode 输入目录（未指定 gcode 时使用）")
    parser.add_argument("--output-dir", type=str, default="", help="npz 输出目录")
    parser.add_argument("--out", type=str, default="", help="输出 npz 文件路径（优先级最高）")
    parser.add_argument("--dt", type=float, default=0.004, help="采样周期秒，默认 4ms")
    parser.add_argument("--chunk-size", type=int, default=100000, help="npz 分片行数，默认 100000")
    args = parser.parse_args(argv)

    input_dir = args.input_gcode_dir or os.path.join(args.data_root, "input_gcode")
    output_dir = args.output_dir or os.path.join(args.data_root, "output_npz")

    gcode_path = args.gcode or _select_default_gcode_file(input_dir)
    if not gcode_path or not os.path.exists(gcode_path):
        print(f"[ERROR] 未找到 GCode 文件: {gcode_path or '(empty)'}", file=sys.stderr)
        return 1

    if args.out:
        output_path = args.out
        out_dir = os.path.dirname(output_path) or "."
    else:
        out_dir = output_dir
        output_path = _default_output_path(gcode_path, out_dir)

    os.makedirs(out_dir, exist_ok=True)

    lines = load_gcode_lines(gcode_path)
    parsed = parse_gcode_lines(lines)
    export_npz(
        parsed,
        output_path,
        dt=args.dt,
        chunk_size=args.chunk_size,
    )
    print(f"[INFO] 导出完成: {output_path} (npz, chunk={args.chunk_size})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
