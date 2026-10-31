"""Pre-flight safety checks before any arm movement (read-only).

Never opens a serial port and imports neither arm.py nor any hardware
SDK. The arm port is found by USB ID via usb_devices.py, and the SDK
channel is read via sdk_config.py. All check logic takes injected
values so pytest runs without hardware.
"""

from __future__ import annotations

import getpass
import grp
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass

from muse_qc.arm.sdk_config import read_sdk_channel
from muse_qc.checks.usb_devices import ARM_USB_PID, ARM_USB_VID, find_serial_ports

_SUSPICIOUS_PROGRAM_RE = re.compile(r"(?:^|[-_./])(rebot|ros|motorbridge)", re.IGNORECASE)
SKIPPED_NO_PORT = "skipped: no arm port"


@dataclass(frozen=True)
class CheckResult:
    """Outcome of one pre-flight check."""
    name: str
    ok: bool
    detail: str


def program_basename(command: str) -> str:
    """Return the basename of the first whitespace-separated element."""
    first = command.strip().split(None, 1)
    if not first:
        return ""
    token = first[0].strip("\"'")
    base = token.rsplit("/", 1)[-1].rsplit("\\", 1)[-1]
    return base


def candidate_names(command: str) -> list[str]:
    """Return matchable names from every command-line element plus -m module."""
    tokens = command.strip().split()
    names = [program_basename(token) for token in tokens]
    for index, token in enumerate(tokens):
        if token.strip("\"'") == "-m" and index + 1 < len(tokens):
            names.append(tokens[index + 1].strip("\"'").split(".")[0])
    return names


def find_suspicious_processes(ps_output: str, own_pid: int) -> list[str]:
    """Return ps lines with suspicious program names, excluding own PID."""
    found: list[str] = []
    for line in ps_output.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        parts = stripped.split(None, 1)
        if len(parts) < 2:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue  # Header line such as "PID COMMAND".
        if pid == own_pid:
            continue
        if any(_SUSPICIOUS_PROGRAM_RE.search(n) for n in candidate_names(parts[1])):
            found.append(stripped)
    return found


def check_competing_processes(ps_output: str | None, own_pid: int) -> CheckResult:
    """FAIL when another process could be driving the arm or ps failed."""
    if ps_output is None:
        return CheckResult(name="competing-processes", ok=False,
                            detail="not verifiable: ps failed")
    found = find_suspicious_processes(ps_output, own_pid)
    if found:
        return CheckResult(name="competing-processes", ok=False,
                            detail=f"suspicious processes running: {'; '.join(found)}")
    return CheckResult(name="competing-processes", ok=True,
                        detail="no rebot/ros/motorbridge process found")


def check_arm_usb(ports: list[str]) -> CheckResult:
    """OK with exactly one arm port found by USB ID, else FAIL."""
    if len(ports) == 1:
        return CheckResult(name="arm-usb", ok=True,
                            detail=f"{ports[0]} found by USB ID {ARM_USB_VID}:{ARM_USB_PID}")
    if not ports:
        return CheckResult(name="arm-usb", ok=False,
                            detail=f"arm not found by USB ID {ARM_USB_VID}:{ARM_USB_PID}")
    return CheckResult(name="arm-usb", ok=False,
                        detail=f"multiple arms found by USB ID {ARM_USB_VID}:{ARM_USB_PID}: "
                        + ", ".join(ports))


def _skipped(name: str) -> CheckResult:
    """Port check outcome when no arm port was found by USB ID."""
    return CheckResult(name=name, ok=False, detail=SKIPPED_NO_PORT)


def check_sdk_port(channel: str | None, port: str | None) -> CheckResult:
    """OK only when the SDK channel is the arm port found by USB ID."""
    if port is None:
        return _skipped("sdk-port")
    if channel is None:
        return CheckResult(name="sdk-port", ok=False,
                            detail="not verifiable: SDK channel unreadable")
    if not channel.startswith("/dev/"):
        return CheckResult(name="sdk-port", ok=False,
                            detail=f"SDK channel is not a serial device: {channel}")
    if os.path.realpath(channel) == os.path.realpath(port):
        return CheckResult(name="sdk-port", ok=True,
                            detail=f"SDK channel {channel} matches arm port {port}")
    return CheckResult(name="sdk-port", ok=False,
                        detail=f"SDK channel {channel} differs from arm port {port}")


def check_device_exists(exists: bool, port: str) -> CheckResult:
    """FAIL when the serial device node is missing."""
    if exists:
        return CheckResult(name="device-exists", ok=True, detail=f"{port} exists")
    return CheckResult(name="device-exists", ok=False, detail=f"{port} is missing")


def check_port_access(can_read: bool, can_write: bool, in_dialout: bool) -> CheckResult:
    """FAIL when the user cannot read/write the port or misses dialout."""
    problems: list[str] = []
    if not can_read:
        problems.append("not readable by this user")
    if not can_write:
        problems.append("not writable by this user")
    if not in_dialout:
        problems.append("user is not in group dialout")
    if problems:
        return CheckResult(name="port-access", ok=False, detail="; ".join(problems))
    return CheckResult(name="port-access", ok=True,
                        detail="readable, writable, user in group dialout")


def parse_fuser_pids(output: str) -> list[str]:
    """Extract PIDs from fuser stdout/stderr text (bare numbers only)."""
    if ":" in output:
        output = output.split(":", 1)[1]
    return [token for token in re.split(r"\s+", output) if token.isdigit()]


def parse_lsof_pids(output: str) -> list[str]:
    """Extract PIDs from lsof output (supports -t and default table)."""
    pids: list[str] = []
    for line in output.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("COMMAND"):
            continue
        fields = stripped.split()
        if len(fields) >= 2 and fields[1].isdigit():
            pids.append(fields[1])
        elif fields[0].isdigit():
            pids.append(fields[0])
    return pids


def check_port_holder(
    fuser_output: str | None, lsof_output: str | None, port: str
) -> CheckResult:
    """FAIL when another process holds the port (fuser preferred, else lsof)."""
    if fuser_output is not None:
        pids = parse_fuser_pids(fuser_output)
        if pids:
            return CheckResult(name="port-holder", ok=False,
                                detail=f"{port} held by PIDs (fuser): {', '.join(pids)}")
        return CheckResult(name="port-holder", ok=True,
                            detail=f"no process holds {port} (fuser)")
    if lsof_output is not None:
        pids = parse_lsof_pids(lsof_output)
        if pids:
            return CheckResult(name="port-holder", ok=False,
                                detail=f"{port} held by PIDs (lsof): {', '.join(pids)}")
        return CheckResult(name="port-holder", ok=True,
                            detail=f"no process holds {port} (lsof)")
    return CheckResult(name="port-holder", ok=False,
                        detail="not verifiable: neither fuser nor lsof is available")


def run_all_checks(
    ps_output: str | None,
    own_pid: int,
    arm_ports: list[str],
    device_exists: bool,
    can_read: bool,
    can_write: bool,
    in_dialout: bool,
    fuser_output: str | None,
    lsof_output: str | None,
    port: str | None,
    sdk_channel: str | None,
) -> tuple[list[CheckResult], bool]:
    """Run every check on injected values; True only when all pass."""
    results = [check_competing_processes(ps_output, own_pid), check_arm_usb(arm_ports),
                check_sdk_port(sdk_channel, port)]
    if port is None:
        results.extend([_skipped("device-exists"), _skipped("port-access"),
                         _skipped("port-holder")])
    else:
        results.extend([check_device_exists(device_exists, port),
                         check_port_access(can_read, can_write, in_dialout),
                         check_port_holder(fuser_output, lsof_output, port)])
    return results, all(item.ok for item in results)


def _run_command(args: list[str]) -> str | None:
    """Run a read-only probe; None when the probe could not run at all."""
    try:
        completed = subprocess.run(args, capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return None
    return (completed.stdout or "") + (completed.stderr or "")


def get_user_group_names() -> list[str]:
    """Return group names of the current user (read-only lookup)."""
    names: set[str] = set()
    try:
        for gid in os.getgroups():
            try:
                names.add(grp.getgrgid(gid).gr_name)
            except KeyError:
                continue
    except OSError:
        pass
    try:
        user = getpass.getuser()
        try:
            dialout = grp.getgrnam("dialout")
            if user in dialout.gr_mem:
                names.add("dialout")
        except KeyError:
            pass
    except OSError:
        pass
    return sorted(names)


def main() -> int:
    """Collect read-only system state, print checks, return exit code."""
    own_pid = os.getpid()
    ps_output = _run_command(["ps", "-eo", "pid,args"])
    arm_ports = find_serial_ports(ARM_USB_VID, ARM_USB_PID)
    port = arm_ports[0] if len(arm_ports) == 1 else None
    sdk_channel = read_sdk_channel()

    exists = can_read = can_write = False
    in_dialout = "dialout" in get_user_group_names()
    fuser_output: str | None = None
    lsof_output: str | None = None
    if port is not None:
        exists = os.path.exists(port)
        can_read = os.access(port, os.R_OK)
        can_write = os.access(port, os.W_OK)
        if shutil.which("fuser") is not None:
            fuser_output = _run_command(["fuser", port])
        if fuser_output is None and shutil.which("lsof") is not None:
            lsof_output = _run_command(["lsof", "-t", port])

    results, overall = run_all_checks(
        ps_output=ps_output,
        own_pid=own_pid,
        arm_ports=arm_ports,
        device_exists=exists,
        can_read=can_read,
        can_write=can_write,
        in_dialout=in_dialout,
        fuser_output=fuser_output,
        lsof_output=lsof_output,
        port=port,
        sdk_channel=sdk_channel,
    )
    for item in results:
        label = "OK" if item.ok else "FAIL"
        print(f"{item.name}: {label} - {item.detail}")
    print(f"overall: {'OK' if overall else 'FAIL'}")
    return 0 if overall else 1


if __name__ == "__main__":
    sys.exit(main())
