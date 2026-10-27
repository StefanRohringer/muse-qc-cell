"""Tests for preflight.py. No hardware is touched."""

from __future__ import annotations

import os

import preflight
from preflight import (
    candidate_names,
    check_competing_processes,
    check_device_exists,
    check_port_access,
    check_port_holder,
    find_suspicious_processes,
    parse_fuser_pids,
    parse_lsof_pids,
    run_all_checks,
)

CLEAN_PS = "  PID COMMAND\n    1 /sbin/init\n  100 python preflight.py\n"

REBOT_PS = "  PID COMMAND\n    1 /sbin/init\n  210 /opt/rebot-driver\n"

ROS_PS = "  PID COMMAND\n    1 /sbin/init\n  311 /opt/ros/humble/bin/ros2 run demo\n"

BRIDGE_PS = "  PID COMMAND\n    1 /sbin/init\n  420 /usr/bin/MotorBridge\n"


def test_clean_process_list_is_ok():
    result = check_competing_processes(CLEAN_PS, own_pid=100)
    assert result.ok


def test_rebot_process_is_fail():
    result = check_competing_processes(REBOT_PS, own_pid=1)
    assert not result.ok


def test_ros_process_is_fail():
    result = check_competing_processes(ROS_PS, own_pid=1)
    assert not result.ok


def test_motorbridge_match_is_case_insensitive():
    result = check_competing_processes(BRIDGE_PS, own_pid=1)
    assert not result.ok


def test_own_pid_is_excluded():
    ps = "  PID COMMAND\n  555 /opt/rebot-driver\n"
    assert find_suspicious_processes(ps, own_pid=555) == []
    assert check_competing_processes(ps, own_pid=555).ok


def test_other_pid_with_same_command_is_fail():
    ps = "  PID COMMAND\n  555 /opt/rebot-driver\n"
    assert not check_competing_processes(ps, own_pid=999).ok


def test_harmless_names_with_substrings_are_ok():
    for name in ("gnome-shell", "chrome", "process"):
        ps = f"  PID COMMAND\n  700 {name}\n"
        assert find_suspicious_processes(ps, own_pid=1) == []
        assert check_competing_processes(ps, own_pid=1).ok


def test_whole_word_names_are_fail():
    for name in ("rosmaster", "rebot_driver", "motorbridge"):
        ps = f"  PID COMMAND\n  700 {name}\n"
        assert not check_competing_processes(ps, own_pid=1).ok


def test_own_pid_excluded_for_required_names():
    for name in ("rosmaster", "rebot_driver", "motorbridge"):
        ps = f"  PID COMMAND\n  700 {name}\n"
        assert find_suspicious_processes(ps, own_pid=700) == []


def test_directory_part_alone_is_ok():
    ps = "  PID COMMAND\n  210 /opt/rebot/driver\n"
    assert find_suspicious_processes(ps, own_pid=1) == []
    assert check_competing_processes(ps, own_pid=1).ok


def test_arguments_alone_do_not_match():
    ps = "  PID COMMAND\n  420 /usr/bin/python script.py --output /tmp/data\n"
    assert find_suspicious_processes(ps, own_pid=1) == []
    assert check_competing_processes(ps, own_pid=1).ok


def test_module_invocation_is_fail():
    ps = "  PID COMMAND\n  420 /usr/bin/python -m MotorBridge.server\n"
    assert find_suspicious_processes(ps, own_pid=1) != []
    assert not check_competing_processes(ps, own_pid=1).ok


def test_innocent_module_is_ok():
    ps = "  PID COMMAND\n  420 /usr/bin/python -m pytest tests/\n"
    assert find_suspicious_processes(ps, own_pid=1) == []
    assert check_competing_processes(ps, own_pid=1).ok


def test_candidate_names_covers_every_element_and_module():
    names = candidate_names("/usr/bin/python -m MotorBridge.server")
    assert "python" in names
    assert "MotorBridge.server" in names
    assert "MotorBridge" in names


def test_fuser_failure_falls_back_to_lsof():
    assert check_port_holder(fuser_output=None, lsof_output="").ok
    assert not check_port_holder(fuser_output=None, lsof_output="1234\n").ok


def test_missing_device_is_fail():
    assert not check_device_exists(False).ok


def test_existing_device_is_ok():
    assert check_device_exists(True).ok


def test_port_access_ok_when_all_good():
    assert check_port_access(True, True, True).ok


def test_port_access_fail_without_read():
    assert not check_port_access(False, True, True).ok


def test_port_access_fail_without_write():
    assert not check_port_access(True, False, True).ok


def test_port_access_fail_without_dialout():
    assert not check_port_access(True, True, False).ok


def test_fuser_empty_means_free():
    result = check_port_holder(fuser_output="", lsof_output=None)
    assert result.ok


def test_fuser_pid_means_held():
    result = check_port_holder(
        fuser_output="/dev/ttyACM0: 1234", lsof_output=None
    )
    assert not result.ok


def test_lsof_used_when_fuser_missing_and_free():
    result = check_port_holder(fuser_output=None, lsof_output="")
    assert result.ok


def test_lsof_pid_means_held():
    result = check_port_holder(fuser_output=None, lsof_output="1234\n")
    assert not result.ok


def test_missing_tools_is_not_ok():
    result = check_port_holder(fuser_output=None, lsof_output=None)
    assert not result.ok
    assert "not verifiable" in result.detail


def test_fuser_preferred_over_lsof():
    result = check_port_holder(fuser_output="", lsof_output="9999\n")
    assert result.ok


def test_parse_fuser_pids():
    assert parse_fuser_pids("/dev/ttyACM0: 1234 5678") == ["1234", "5678"]
    assert parse_fuser_pids("") == []
    assert parse_fuser_pids("Specified filename /dev/ttyACM0 does not exist.\n") == []
    assert parse_fuser_pids("/dev/ttyACM0:") == []


def test_parse_lsof_pids():
    assert parse_lsof_pids("1234\n") == ["1234"]
    table = "COMMAND PID USER\npython 4321 me\n"
    assert parse_lsof_pids(table) == ["4321"]


def test_run_all_checks_ok_only_when_everything_ok():
    _, overall = run_all_checks(
        ps_output=CLEAN_PS,
        own_pid=100,
        device_exists=True,
        can_read=True,
        can_write=True,
        in_dialout=True,
        fuser_output="",
        lsof_output=None,
    )
    assert overall


def test_run_all_checks_fail_on_competitor():
    _, overall = run_all_checks(
        ps_output=REBOT_PS,
        own_pid=1,
        device_exists=True,
        can_read=True,
        can_write=True,
        in_dialout=True,
        fuser_output="",
        lsof_output=None,
    )
    assert not overall


def test_run_all_checks_fail_when_port_held():
    _, overall = run_all_checks(
        ps_output=CLEAN_PS,
        own_pid=100,
        device_exists=True,
        can_read=True,
        can_write=True,
        in_dialout=True,
        fuser_output="/dev/ttyACM0: 77",
        lsof_output=None,
    )
    assert not overall


def test_run_all_checks_fail_when_device_missing():
    _, overall = run_all_checks(
        ps_output=CLEAN_PS,
        own_pid=100,
        device_exists=False,
        can_read=False,
        can_write=False,
        in_dialout=True,
        fuser_output="",
        lsof_output=None,
    )
    assert not overall


def test_preflight_imports_no_hardware_modules():
    path = os.path.join(os.path.dirname(__file__), "..", "preflight.py")
    with open(path, encoding="utf-8") as handle:
        source = handle.read()
    for forbidden in (
        "from arm import",
        "import arm",
        "motorbridge",
        "rebotarm",
        "reBotArm",
    ):
        if forbidden == "motorbridge":
            # Allowed once as a suspicious process name, not as an import.
            assert "import motorbridge" not in source
            assert "from motorbridge" not in source
            continue
        assert forbidden not in source


def test_preflight_never_opens_the_port():
    path = os.path.join(os.path.dirname(__file__), "..", "preflight.py")
    with open(path, encoding="utf-8") as handle:
        source = handle.read()
    assert "open(PORT" not in source
    assert "open(\"/dev" not in source
    assert "import serial" not in source
    assert "from serial" not in source
    assert "pyserial" not in source
    assert preflight.PORT == "/dev/ttyACM0"
