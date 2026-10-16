"""Diagnostic CLI outcomes use substitute devices; no USB is opened."""
from pathlib import Path
import runpy
from unittest.mock import MagicMock

import pytest


SCRIPTS = Path(__file__).resolve().parents[2] / "python_script"


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def scanner(monkeypatch, replies=(1, 2)):
    script = runpy.run_path(str(SCRIPTS / "scan_motors_online.py"))
    arm = MagicMock()
    arm.left.can_ids, arm.right.can_ids = [1], [2]
    arm.probe_status.return_value = replies
    monkeypatch.setattr(script["Arm"], "from_yaml", lambda _: arm)
    script["main"].__globals__["time"] = Clock()
    return script["main"], arm


def test_passive_scan_closes_before_success(monkeypatch, capsys):
    main, arm = scanner(monkeypatch)
    arm.disable.side_effect = lambda: print("CLOSED")
    assert main(["--duration", "0.2", "--min-hits", "2"]) == 0
    output = capsys.readouterr().out
    assert output.index("CLOSED") < output.index("PASS:")
    arm.connect_passive.assert_called_once()
    arm.enable.assert_not_called()
    arm._arm.send_zero_mit_all.assert_not_called()
    arm.states.assert_not_called()


def test_scan_shutdown_failure_prevents_success(monkeypatch, capsys):
    main, arm = scanner(monkeypatch)
    arm.disable.side_effect = RuntimeError("disable unconfirmed")
    assert main(["--duration", "0.2", "--min-hits", "1"]) == 2
    output = capsys.readouterr().out
    assert "disable unconfirmed" in output and "PASS:" not in output


def test_scan_preserves_query_and_shutdown_errors(monkeypatch, capsys):
    main, arm = scanner(monkeypatch)
    arm.probe_status.side_effect = RuntimeError("query failed")
    arm.disable.side_effect = RuntimeError("close failed")
    assert main(["--duration", "0.2"]) == 2
    output = capsys.readouterr().out
    assert "query failed" in output and "close failed" in output and "PASS:" not in output


def test_scan_only_counts_selected_distinct_replies(monkeypatch, capsys):
    main, arm = scanner(monkeypatch, replies=(1, 1, 99))
    assert main(["--duration", "0.01", "--min-hits", "2"]) == 3
    output = capsys.readouterr().out
    assert "summary: 0/2 online" in output and "PASS:" not in output
    arm.disable.assert_called_once()


@pytest.mark.parametrize("name", ["read_states.py", "link_test.py"])
def test_active_diagnostic_retains_shutdown_failure(monkeypatch, capsys, name):
    script = runpy.run_path(str(SCRIPTS / name))
    arm = MagicMock()
    arm.states.side_effect = RuntimeError("read failed")
    arm.disable.side_effect = RuntimeError("close failed")
    monkeypatch.setattr(script["Arm"], "from_yaml", lambda _: arm)
    assert script["main"]() == 2
    output = capsys.readouterr().out
    assert "read failed" in output and "close failed" in output and "PASS:" not in output


@pytest.mark.parametrize("name", ["read_states.py", "link_test.py"])
def test_active_diagnostic_interruption_still_checks_shutdown(monkeypatch, capsys, name):
    script = runpy.run_path(str(SCRIPTS / name))
    arm = MagicMock()
    arm.states.side_effect = KeyboardInterrupt()
    arm.disable.side_effect = RuntimeError("close failed")
    monkeypatch.setattr(script["Arm"], "from_yaml", lambda _: arm)
    assert script["main"]() == 2
    assert "close failed" in capsys.readouterr().out
