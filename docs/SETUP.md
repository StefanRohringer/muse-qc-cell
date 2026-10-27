# Setup

One machine holds both this project and the Seeed SDK checkout.
The SDK lives outside this repository; only the dependency pointer lives here.

## Prerequisites

- `uv` (install from https://astral.sh/uv/install.sh)
- A `.env` file with `MODEL_API_KEY` (loaded by the program, never printed)
- No hardware access is needed for setup. Never call `connect()` here.

## Why Python 3.11

The official Seeed SDK requires Python `>=3.10,<3.12`
(Pinocchio/`pin` has no Python 3.12 build), so this project
was switched from Python 3.12 to Python 3.11:

- `.python-version` contains `3.11`
- `pyproject.toml` declares `requires-python = ">=3.11"`
- `uv.lock` was regenerated (it now also lists the cp311 motorbridge wheels)

## 1. Clone the Seeed SDK (outside this repo)

```bash
cd ~/meta-hackathon
git clone https://github.com/Seeed-Projects/reBotArm_control_py.git seeed-sdk
```

Pinned commit: `d4f3591bc0183de03f3ae8651298db0269aadfdd` (main, 2026-10-21).
Nothing from the SDK is copied into this repository.

## 2. Apply the packaging workaround (in the SDK clone)

Installing the SDK as-is fails with the upstream setuptools error
`Multiple top-level packages discovered in a flat-layout`
(`urdf`, `config` and `reBotArm_control_py` are auto-discovered).
Fix it with an explicit package discovery rule, appended to
`~/meta-hackathon/seeed-sdk/pyproject.toml`:

```toml
[tool.setuptools.packages.find]
include = ["reBotArm_control_py*"]
```

Verify with `uv build` inside the SDK clone; it must produce
a wheel and an sdist without errors.

## 3. Select the B601-DM hardware config (in the SDK clone)

`~/meta-hackathon/seeed-sdk/config/rebotarm.yaml` defaults to the RS arm.
Point it at the DM arm used here:

```yaml
hardware_yaml: "rebotarm_dm.yaml"
```

See the Seeed Wiki page "reBot Arm B601-DM Pinocchio & MeshCat" for the
config file layout.

## 4. Install the SDK editable into the project environment

From this project root:

```bash
rm -rf .venv
uv sync --python 3.11
uv add --editable ../seeed-sdk
```

This records an editable path dependency in `pyproject.toml`
(`[tool.uv.sources]`) and pins it in `uv.lock`, so plain `uv sync`
keeps the SDK installed instead of removing it again.

## 5. Verify (import only, no hardware)

```bash
uv sync
uv pip show rebotarm-control-py
uv run python -c "import reBotArm_control_py"
```

The import must succeed. Do not call `connect()` and do not touch
any hardware; hardware tests are started by the human only.

## 6. Run the project checks

```bash
uv run python --version  # must report Python 3.11.x
```

`uv run pytest` is the project test command (see AGENTS.md);
it runs the suite in `tests/` (tests/test_arm.py, tests/test_diagnose.py, tests/test_preflight.py, tests/test_safety.py, tests/test_sdk_config.py, tests/test_usb_devices.py).
All tests run without the Seeed SDK; one test in tests/test_safety.py reads the real joint limits and is skipped when the SDK is not installed.

## 7. Trying muse_qc/arm.py on the real arm (by hand only)

This step touches hardware, so only the human runs it, never CI and
never an agent. `muse_qc/arm.py` only reads: it cannot enable, zero or move
the arm (enable, zero, mode and send commands are unreachable through
it), but `connect()` still opens the real bus. From the project root:

```bash
uv run python -c "
from muse_qc.arm import Arm
arm = Arm()
arm.connect()
try:
    for joint in arm.read_joints():
        print(f'{joint.name}: {joint.degrees:+.2f} deg ({joint.radians:+.4f} rad)')
    gripper = arm.read_gripper()
    print(f'{gripper.name}: {gripper.degrees:+.2f} deg ({gripper.radians:+.4f} rad)')
finally:
    arm.disconnect()
"
```

A `JointStateUnavailable` error means a joint reported exactly `0.0`
(or a non-finite value): the SDK substitutes `0.0` for missing
feedback, so `muse_qc/arm.py` never reports `0.0` as an angle. Check the bus
connection and the motor power, then retry.

## 8. Finding the arm by USB ID (by hand only)

This step is run by the human only, never by an agent. The arm port is
never a fixed `/dev/ttyACM0`: `muse_qc/preflight.py` finds it by USB vendor and
product ID through `muse_qc/usb_devices.py` (read-only sysfs lookup, the serial
port itself is never opened).

How to read the IDs (arm power supply off, only USB connected):

```bash
ls -l /dev/serial/by-id/
udevadm info -q property -n /dev/ttyACM0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID"
```

Expected for the arm: `ID_VENDOR_ID=2e88`, `ID_MODEL_ID=4603`.
(The gripper Pico reports `2e8a`, which is a different device and is
ignored by the lookup.)

How to run the check, from the project root:

```bash
uv run python -m muse_qc.preflight
```

Expected result: `arm-usb: OK` with the found path. With the USB cable
unplugged: `arm-usb: FAIL` (`arm not found by USB ID 2e88:4603`), and the
port checks report `skipped: no arm port` instead of probing.

`muse_qc/preflight.py` now also checks that the SDK opens the same device:
`sdk-port` compares the SDK channel from `muse_qc/sdk_config.py`
(`config/rebotarm.yaml` via `hardware_yaml`) with the arm port found by
USB ID. Expected: `sdk-port: OK`. It FAILs when the channel is
unreadable, is not a `/dev` path (for example `can0`), or points to a
different device; without an arm port it reports `skipped: no arm port`.

Never write the device serial number into the repository.

## 9. Table camera test (by hand only)

This step opens the real camera, so only the human runs it, never CI
and never an agent. `muse_qc/camera_top.py` finds the table camera by
USB vendor and product ID through `muse_qc/usb_devices.py`
(read-only sysfs lookup under `/sys/class/video4linux`, capture nodes
only), opens exactly one matching device with the V4L2 backend, requests
MJPG at 1920x1080, discards 45 warm-up frames (about 1.5 s at 30 fps,
letting the camera settle after the settings are applied), then reads
the returned frame and saves it. The camera runs in MJPG at 30 fps. It
imports no arm code and moves nothing. From the project root:

```bash
uv run python -m muse_qc.camera_top
```

Expected result: one new file `recordings/table_<YYYYmmdd_HHMMSS>.png`
at 1920x1080 and one printed line with path, width and height, for
example:

```text
recordings/table_20261027_123456.png 1920x1080
```

With the camera unplugged the command fails with
`table camera not found by USB ID 145f:0203`; with several matching
devices it fails with `several table cameras found` and never falls
back to a fixed index. When the real frame size is not 1920x1080 the
command fails with `unexpected frame size WxH` and never silently falls
back to a lower resolution.

> Mounting: in this cell the table camera (USB ID 145f:0203) is mounted upside down above the table. camera_top.py therefore rotates every frame by 180 degrees (TABLE_CAM_ROTATE_180 = True). If your camera is mounted normally, set it to False.

Fixed settings: every start applies manual exposure mode
(auto_exposure 1), exposure_time_absolute 450, gain 0, brightness 160,
contrast 150, automatic white balance off (white_balance_automatic 0)
at 3200 K. The values are set after the format and size and before the
warm-up frames, then read back and printed in the same line as the
image path. The camera keeps its last settings even after being
unplugged, so values set earlier by hand or by another program would
carry over; applying the fixed values on every start makes the result
reproducible.

How to check them by hand (the device node may differ):

```bash
v4l2-ctl -d /dev/video10 --get-ctrl=auto_exposure,exposure_time_absolute,gain,brightness,contrast,white_balance_automatic,white_balance_temperature
```

## 10. Table markers (print by hand)

This step uses only the printer, so the human runs it by hand, never CI
and never an agent. `muse_qc/make_markers.py` writes one A4 page with
the four ArUco table markers (IDs 0 to 3, shared constants in
`muse_qc/table_markers.py`). From the project root:

```bash
uv run python -m muse_qc.make_markers
```

Expected result: one printed line with the path `docs/table_markers.svg`.
The file is part of the repository, so it can be printed straight from
there. It can be generated again with the command above.

Print the page at 100 % (actual size, no scaling to fit the page).
Then check with a ruler: the scale line must measure exactly 100 mm
and every black marker square exactly 40 mm. Cut out the four markers
and stick one marker per table corner. Which ID ends up in which corner
does not matter: the detection added later finds the corner of each ID
by itself.

## Safety notes

Read-only review of the SDK file
`reBotArm_control_py/actuator/rebotarm.py` (SDK clone at
`~/meta-hackathon/seeed-sdk`) and of `muse_qc/arm.py`. No hardware was touched.

- The constructor `RebotArm.__init__` (`rebotarm.py:529-550`) starts no
  thread and sends nothing. It only loads the config, stores channel,
  rate and joint definitions, sets `_running = False` (`:544`) and
  `_ctrl_thread = None` (`:545`), then builds the joint groups.
- `connect()` (`rebotarm.py:552-557`) starts no thread and sends no
  motion commands. It only registers the motors via `_setup_motors()`
  (`rebotarm.py:564-582`) and sets `_connected = True`.
- The control loop runs only after an explicit `start_control_loop()`
  call (`rebotarm.py:785-800`). That method creates the single
  background thread (`threading.Thread`, `rebotarm.py:795-799`) and runs
  the caller-supplied callback at `dt = 1.0 / rate`
  (`_control_loop_impl`, `rebotarm.py:802-814`); the default rate comes
  from the hardware config (`load_cfg`, `rebotarm.py:113`). The loop
  itself sends nothing; motion frames only come from what the callback
  calls (`send_mit`, `send_pos_vel` or `send_vel`,
  `rebotarm.py:382-441`).
- `get_positions()` (`rebotarm.py:465-485`) sends only explicit feedback
  request frames plus receive-queue polling (`_request_feedback`,
  `rebotarm.py:456-463`; `_poll_feedback`, `rebotarm.py:445-454`). It
  starts no thread and commands no motion. This is the path `muse_qc/arm.py`
  uses for `read_joints()` and `read_gripper()`.
- `disconnect()` (`rebotarm.py:738-750`) stops the control loop
  (`stop_control_loop`, `rebotarm.py:816-820`), then calls
  `disable_all()` (`rebotarm.py:658-664`, per group
  `rebotarm.py:294-303`), waits, and shuts the bus controllers down
  (`shutdown()` / `close()`, `rebotarm.py:744-747`). After
  `disable_all()` the motors no longer hold position, so the arm can sag
  under its own weight.
- The `muse_qc/arm.py` wrapper adds no thread and no bus traffic of its own:
  `Arm.__init__` (`muse_qc/arm.py:78-80`) only stores the factory,
  `Arm.connect` (`muse_qc/arm.py:87-93`) only calls `sdk.connect()`, and
  `Arm.disconnect` (`muse_qc/arm.py:95-102`) only calls `sdk.disconnect()`.
- Not verified: the exact bytes the `motorbridge` package
  (`Controller.disable_all`, `shutdown`, `close`, `send_*`) puts on the
  bus. That package was outside this review.
