"""Pre-flight safety checks before any arm movement.

Read-only script: it never opens /dev/ttyACM0, sends nothing, and
imports neither arm.py nor any hardware SDK. All check logic takes
command output as parameters so pytest can run without hardware.
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

PORT = "/dev/ttyACM0"
SUSPICIOUS_SUBSTRINGS = ("rebot", "ros", "motorbridge")


@dataclass(frozen=True)
class CheckResult:
    """Outcome of one pre-flight check."""

    name: str
    ok: bool
    detail: str


def find_suspicious_processes(ps_output: str, own_pid: int) -> list[str]:
    """Return ps lines with suspicious names, excluding our own PID."""
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
        command = parts[1].lower()
        if any(word in command for word in SUSPICIOUS_SUBSTRINGS):
            found.append(stripped)
    return found


def check_competing_processes(ps_output: str, own_pid: int) -> CheckResult:
    """FAIL when another process could be driving the arm."""
    found = find_suspicious_processes(ps_output, own_pid)
    if found:
        return CheckResult(
            name="competing-processes",
            ok=False,
            detail=f"suspicious processes running: {'; '.join(found)}",
        )
    return CheckResult(
        name="competing-processes",
        ok=True,
        detail="no rebot/ros/motorbridge process found",
    )


def check_device_exists(exists: bool, port: str = PORT) -> CheckResult:
    """FAIL when the serial device node is missing."""
    if exists:
        return CheckResult(
            name="device-exists", ok=True, detail=f"{port} exists"
        )
    return CheckResult(
        name="device-exists", ok=False, detail=f"{port} is missing"
    )


def check_port_access(
    can_read: bool, can_write: bool, in_dialout: bool
) -> CheckResult:
    """FAIL when the user cannot read/write the port or misses dialout."""
    problems: list[str] = []
    if not can_read:
        problems.append("not readable by this user")
    if not can_write:
        problems.append("not writable by this user")
    if not in_dialout:
        problems.append("user is not in group dialout")
    if problems:
        return CheckResult(
            name="port-access", ok=False, detail="; ".join(problems)
        )
    return CheckResult(
        name="port-access",
        ok=True,
        detail="readable, writable, user in group dialout",
    )


def parse_fuser_pids(output: str) -> list[str]:
    """Extract PIDs from fuser stdout/stderr text.

    Only bare numeric tokens count as PIDs, so device names such as
    ttyACM0 and error messages never produce false hits.
    """
    if ":" in output:
        output = output.split(":", 1)[1]
    return [token for token in re.split(r"\s+", output) if token.isdigit()]


def parse_lsof_pids(output: str) -> list[str]:
    """Extract PIDs from lsof output (supports -t and default table)."""
    pids: list[str] = []
    for line in output.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("COMMAND"):
            continue
        fields = stripped.split()
        if len(fields) >= 2 and fields[1].isdigit():
            pids.append(fields[1])
        elif fields[0].isdigit():
            pids.append(fields[0])
    return pids


def check_port_holder(
    fuser_output: str | None, lsof_output: str | None, port: str = PORT
) -> CheckResult:
    """FAIL when another process holds the port.

    fuser is preferred; lsof is used only when fuser output is None
    (tool missing). Both None means not verifiable, which is a FAIL.
    """
    if fuser_output is not None:
        pids = parse_fuser_pids(fuser_output)
        if pids:
            return CheckResult(
                name="port-holder",
                ok=False,
                detail=f"{port} held by PIDs (fuser): {', '.join(pids)}",
            )
        return CheckResult(
            name="port-holder",
            ok=True,
            detail=f"no process holds {port} (fuser)",
        )
    if lsof_output is not None:
        pids = parse_lsof_pids(lsof_output)
        if pids:
            return CheckResult(
                name="port-holder",
                ok=False,
                detail=f"{port} held by PIDs (lsof): {', '.join(pids)}",
            )
        return CheckResult(
            name="port-holder",
            ok=True,
            detail=f"no process holds {port} (lsof)",
        )
    return CheckResult(
        name="port-holder",
        ok=False,
        detail="not verifiable: neither fuser nor lsof is available",
    )


def run_all_checks(
    ps_output: str,
    own_pid: int,
    device_exists: bool,
    can_read: bool,
    can_write: bool,
    in_dialout: bool,
    fuser_output: str | None,
    lsof_output: str | None,
    port: str = PORT,
) -> tuple[list[CheckResult], bool]:
    """Run every check on injected values; True only when all pass."""
    results = [
        check_competing_processes(ps_output, own_pid),
        check_device_exists(device_exists, port),
        check_port_access(can_read, can_write, in_dialout),
        check_port_holder(fuser_output, lsof_output, port),
    ]
    return results, all(item.ok for item in results)


def _run_command(args: list[str]) -> str:
    """Run a read-only probe and return combined stdout/stderr text."""
    try:
        completed = subprocess.run(
            args, capture_output=True, text=True, timeout=10
        )
    except (OSError, subprocess.SubprocessError):
        return ""
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

    exists = os.path.exists(PORT)
    can_read = os.access(PORT, os.R_OK)
    can_write = os.access(PORT, os.W_OK)
    in_dialout = "dialout" in get_user_group_names()

    fuser_output: str | None = None
    lsof_output: str | None = None
    if shutil.which("fuser") is not None:
        fuser_output = _run_command(["fuser", PORT])
    elif shutil.which("lsof") is not None:
        lsof_output = _run_command(["lsof", "-t", PORT])

    results, overall = run_all_checks(
        ps_output=ps_output,
        own_pid=own_pid,
        device_exists=exists,
        can_read=can_read,
        can_write=can_write,
        in_dialout=in_dialout,
        fuser_output=fuser_output,
        lsof_output=lsof_output,
    )
    for item in results:
        label = "OK" if item.ok else "FAIL"
        print(f"{item.name}: {label} - {item.detail}")
    print(f"overall: {'OK' if overall else 'FAIL'}")
    return 0 if overall else 1


if __name__ == "__main__":
    sys.exit(main())
