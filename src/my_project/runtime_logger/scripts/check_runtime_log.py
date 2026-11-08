#!/usr/bin/env python3
"""Offline checker for a runtime log run directory.

Read-only by default: parses manifest.json, all records_*.jsonl[.partial]
segments and integrity.json, verifies the six-kind whitelist, per-producer
sequence accounting, sampling intervals and event pairing.

Usage: check_runtime_log.py <run_dir> [--json report.json]
"""

import argparse
import json
import os
import sys
from collections import defaultdict
from pathlib import Path

KIND_WHITELIST = {
    "estat_hot": {"state", "target_steps", "submitted_steps", "last_seq",
                  "ff_sps", "v_sps", "fault"},
    "estat_cold": {"completed_steps", "phase", "hard_fault"},
    "ustat": {"fifo_ovf", "buffer_full", "line_drop", "seq_gap", "parse_err"},
    "latency": {"seq_lag", "ack_delay_ms", "linux_mcu_delay_ms",
                "mcu_robot_delay_ms", "robot_match_error_mm"},
    "event": {"event_type", "event_stage"},
    "e_tx": {"last_extrude_abs"},
}

EXPECTED_KIND_PERIODS_MS = {
    "estat_hot": (200.0, 250.0),
    "estat_cold": (1000.0, 1050.0),
    "ustat": (1000.0, 1050.0),
    "latency": (300.0, 330.0),
}


def read_manifest(run_dir):
    path = Path(run_dir) / "manifest.json"
    if not path.is_file():
        return None, [f"missing manifest: {path}"]
    try:
        return json.loads(path.read_text()), []
    except (OSError, json.JSONDecodeError) as exc:
        return None, [f"unreadable manifest: {exc}"]


def iter_records(run_dir):
    """Yield (lineno, record_dict) for every JSONL record in order."""
    segments = sorted(Path(run_dir).glob("records_*.jsonl*"))
    for seg in segments:
        try:
            with open(seg, "r", encoding="utf-8") as fh:
                for lineno, raw in enumerate(fh, 1):
                    raw = raw.strip()
                    if not raw:
                        continue
                    yield seg.name, lineno, raw
        except OSError:
            continue


def check_whitelist(record):
    kind = record.get("kind")
    allowed = KIND_WHITELIST.get(kind)
    if allowed is None:
        return [f"unknown kind {kind!r}"]
    data = record.get("data", {})
    errors = []
    for key in data:
        if key not in allowed:
            errors.append(f"non-whitelisted field {key!r} in {kind}")
    return errors


def check_sequences(records):
    """Per-producer/per-kind record_seq continuity."""
    errors = []
    by_producer = defaultdict(list)
    for seg, lineno, rec in records:
        pid = rec.get("producer_id", "?")
        by_producer[pid].append((seg, lineno, rec.get("record_seq", -1)))
    for pid, rows in by_producer.items():
        rows.sort(key=lambda r: r[2])
        prev = None
        for seg, lineno, seq in rows:
            if prev is not None and seq <= prev:
                errors.append(
                    f"{pid}: non-monotonic record_seq {seq} after {prev} "
                    f"({seg}:{lineno})")
            prev = seq
        expected = rows[-1][2] - rows[0][2] + 1 if rows else 0
        if expected != len(rows):
            errors.append(
                f"{pid}: record_seq count mismatch "
                f"expected {expected} got {len(rows)}")
    return errors


def check_sampling_intervals(records):
    """Interval statistics per kind using mcu/host timestamps."""
    stats = {}
    for _seg, _lineno, rec in records:
        kind = rec.get("kind")
        ts = rec.get("mcu_sample_us")
        if ts is None:
            ts = rec.get("host_mono_ns")
            scale = 1000.0  # ns -> us
        else:
            scale = 1.0
        if ts is None:
            continue
        key = kind
        stats.setdefault(key, []).append(ts / scale)
    report = {}
    for kind, times in stats.items():
        times.sort()
        intervals = [
            (times[i + 1] - times[i]) / 1000.0
            for i in range(len(times) - 1)
        ]
        if not intervals:
            report[kind] = {"count": 1, "intervals": 0}
            continue
        intervals.sort()
        n = len(intervals)
        expected = EXPECTED_KIND_PERIODS_MS.get(kind)
        exceeded = 0
        if expected:
            _target, limit = expected
            exceeded = sum(1 for v in intervals if v > limit)
        report[kind] = {
            "count": len(times),
            "intervals": n,
            "min_ms": intervals[0],
            "p50_ms": intervals[n // 2],
            "p95_ms": intervals[int(n * 0.95)],
            "max_ms": intervals[-1],
            "exceeded_limit": exceeded,
        }
    return report


def check_event_pairs(records):
    """EV -> EVACK -> EVDONE matching by event_request_id."""
    reqs = defaultdict(lambda: {"EV": 0, "EVACK": 0, "EVDONE": 0, "EVERR": 0})
    errors = []
    for _seg, _lineno, rec in records:
        if rec.get("kind") != "event":
            continue
        stage = rec.get("data", {}).get("event_stage")
        rid = rec.get("event_request_id")
        if rid is not None and stage in reqs[rid]:
            reqs[rid][stage] += 1
    for rid, counts in reqs.items():
        if counts["EVACK"] > 0 and counts["EVDONE"] == 0 and counts["EVERR"] == 0:
            errors.append(f"event {rid}: ACK without DONE/ERR")
    return errors


def check_integrity_counts(run_dir, parsed_records):
    """Cross-check integrity.json counters against parsed data."""
    path = Path(run_dir) / "integrity.json"
    if not path.is_file():
        return [f"missing integrity.json"]
    try:
        integ = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        return [f"unreadable integrity.json: {exc}"]
    errors = []
    written = integ.get("written_records", 0)
    if written != len(parsed_records):
        errors.append(
            f"integrity written_records {written} != parsed {len(parsed_records)}")
    return errors


def check_tail(run_dir):
    """Report .partial files and bad final lines."""
    issues = []
    for seg in sorted(Path(run_dir).glob("records_*.jsonl.partial")):
        issues.append(f"incomplete segment: {seg.name}")
    for seg in sorted(Path(run_dir).glob("records_*.jsonl")):
        try:
            with open(seg, "rb") as fh:
                data = fh.read()
        except OSError:
            continue
        if data and not data.endswith(b"\n"):
            issues.append(f"truncated tail line in {seg.name}")
    return issues


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--json", dest="json_out")
    args = ap.parse_args()

    manifest, errors = read_manifest(args.run_dir)
    records = []
    for seg, lineno, raw in iter_records(args.run_dir):
        try:
            rec = json.loads(raw)
        except json.JSONDecodeError as exc:
            errors.append(f"{seg}:{lineno}: bad JSON: {exc}")
            continue
        records.append((seg, lineno, rec))
        errors.extend(
            f"{seg}:{lineno}: {e}" for e in check_whitelist(rec))

    seq_errors = check_sequences(records)
    intervals = check_sampling_intervals(records)
    event_errors = check_event_pairs(records)
    integ_errors = check_integrity_counts(args.run_dir, records)
    tail_issues = check_tail(args.run_dir)

    report = {
        "run_dir": args.run_dir,
        "manifest": manifest,
        "record_count": len(records),
        "intervals": intervals,
        "sequence_errors": seq_errors,
        "event_errors": event_errors,
        "integrity_errors": integ_errors,
        "tail_issues": tail_issues,
        "other_errors": errors,
        "ok": not (seq_errors or event_errors or integ_errors or tail_issues
                   or errors),
    }
    text = json.dumps(report, indent=2, ensure_ascii=False)
    print(text)
    if args.json_out:
        Path(args.json_out).write_text(text)
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
