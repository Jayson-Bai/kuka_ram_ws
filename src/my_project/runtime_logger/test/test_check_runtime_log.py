import json
import os
import subprocess
import sys
from pathlib import Path

SCRIPT = (
    Path(__file__).resolve().parents[1] / "scripts" / "check_runtime_log.py")


def _write_run(tmp_path, records, complete=True):
    run_dir = tmp_path / "run_test123"
    run_dir.mkdir()
    (run_dir / "manifest.json").write_text(json.dumps({
        "schema_version": 1, "run_id": "test123", "complete": complete}))
    lines = []
    for rec in records:
        lines.append(json.dumps(rec) + "\n")
    (run_dir / "records_000000.jsonl").write_text("".join(lines))
    (run_dir / "integrity.json").write_text(json.dumps({
        "session_complete": complete,
        "written_records": len(records),
        "producers": {}}))
    return run_dir


def _run_checker(run_dir):
    return subprocess.run(
        [sys.executable, str(SCRIPT), str(run_dir)],
        capture_output=True, text=True)


def test_clean_run_passes(tmp_path):
    rec = {
        "schema_version": 1, "run_id": "test123",
        "producer_id": "uart_node", "producer_epoch": 1,
        "record_seq": 0, "capture_seq": 0, "kind": "estat_hot",
        "host_mono_ns": 1000, "valid": True, "tx_ok": True,
        "bytes_written": 0,
        "data": {"state": "RUN", "target_steps": 1,
                 "submitted_steps": 1, "last_seq": 1,
                 "ff_sps": 1.0, "v_sps": 1.0, "fault": "0x0"},
    }
    run_dir = _write_run(tmp_path, [rec])
    result = _run_checker(run_dir)
    assert result.returncode == 0, result.stdout + result.stderr


def test_non_whitelisted_field_fails(tmp_path):
    rec = {
        "schema_version": 1, "run_id": "t",
        "producer_id": "uart_node", "producer_epoch": 1,
        "record_seq": 0, "capture_seq": 0, "kind": "estat_hot",
        "host_mono_ns": 1, "valid": True, "tx_ok": True,
        "bytes_written": 0,
        "data": {"state": "RUN", "target_steps": 1,
                 "submitted_steps": 1, "last_seq": 1,
                 "ff_sps": 1.0, "v_sps": 1.0, "fault": "0x0",
                 "secret": "bad"},
    }
    run_dir = _write_run(tmp_path, [rec])
    result = _run_checker(run_dir)
    assert result.returncode == 1
    assert "non-whitelisted" in result.stdout


def test_seq_gap_detected(tmp_path):
    recs = []
    for i in (0, 1, 3):
        recs.append({
            "schema_version": 1, "run_id": "t",
            "producer_id": "uart_node", "producer_epoch": 1,
            "record_seq": i, "capture_seq": i, "kind": "ustat",
            "host_mono_ns": i, "valid": True, "tx_ok": True,
            "bytes_written": 0,
            "data": {"fifo_ovf": 0, "buffer_full": 0, "line_drop": 0,
                     "seq_gap": 0, "parse_err": 0},
        })
    run_dir = _write_run(tmp_path, recs)
    result = _run_checker(run_dir)
    assert result.returncode == 1
    assert "count mismatch" in result.stdout
