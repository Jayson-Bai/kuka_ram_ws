"""Verify compression, late records and manual export without a ROS/Qt session."""
import gzip
import json

from my_project_ui.diagnostic_log import DiagnosticLog


def test_long_session_keeps_late_fault_and_exports_only_on_request(tmp_path):
    log = DiagnosticLog()
    for i in range(200001):
        log.append({'time_epoch': i * .004, 'host_monotonic_ns': i * 4000000,
                    'source': 'uart', 'kind': 'raw',
                    'detail': {'line': 'TX E 1 1 0.000000',
                               'payload': 'E 1 1 0.000000', 'direction': 'TX'}})
    log.append({'source': 'uart', 'kind': 'raw',
                'detail': {'line': 'RX EWARN late fault'}})
    assert not list(tmp_path.iterdir())
    target = tmp_path / 'manual.jsonl.gz'
    log.export(target)
    with gzip.open(target, 'rt') as stream:
        first = json.loads(next(stream))
        count = 1
        for line in stream:
            last = json.loads(line)
            count += 1
    assert count == 200002
    assert first['time_epoch'] == 0
    assert first['detail'] == {'line': 'TX E 1 1 0.000000'}
    assert last['detail']['line'] == 'RX EWARN late fault'
    assert target.stat().st_size < log.raw_bytes / 3
    # Export must not consume history: subsequent export includes both periods.
    log.append({'source': 'ui', 'kind': 'after_export', 'detail': {}})
    later = tmp_path / 'later.jsonl.gz'
    log.export(later)
    with gzip.open(later, 'rt') as stream:
        assert sum(1 for _ in stream) == 200003
    print(f'compression: {log.raw_bytes} raw bytes -> {later.stat().st_size} gzip bytes')
