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
it runs every file in `tests/`.
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
MJPG at 1920x1080, discards 45 warm-up frames (a short settle time,
letting the camera settle after the settings are applied), then reads
the returned frame and saves it. It
imports no arm code and moves nothing. From the project root:

```bash
uv run python -m muse_qc.camera_top
```

Expected result: one new file `recordings/table_<YYYYmmdd_HHMMSS>.png`
at 1920x1080 and one printed line with path, width, height and the
read-back settings, for example:

```text
recordings/table_20261027_123456.png 1920x1080 auto_exposure=1 exposure=450 gain=0 brightness=160 contrast=150 auto_wb=0 wb_temperature=3200
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
and stick one marker per table corner with the fixed assignment in
`TABLE_MARKER_LAYOUT_MM`: ID 0 bottom left next to the arm (the
origin), ID 1 bottom right, ID 2 top left, ID 3 top right.

## 11. Table marker detection (by hand only)

This step opens the real camera, so only the human runs it, never CI
and never an agent. `muse_qc/detect_markers.py` takes one frame with
`open_table_camera()` from `muse_qc/camera_top.py`, finds the four
ArUco table markers (IDs 0 to 3, shared constants in
`muse_qc/table_markers.py`) with `cv2.aruco.ArucoDetector`, and
requires each ID exactly once: a missing marker raises naming the
missing IDs, a marker seen twice raises naming the duplicated IDs.
Detected IDs outside 0 to 3 are ignored and reported under
`ignored_ids`. From the project root:

```bash
uv run python -m muse_qc.detect_markers
```

Expected result: one line per ID with image corner, center in pixels
and side in pixels, then the scale in px/mm, then the path of one
annotated copy (marker outlines and IDs drawn) at
`recordings/table_markers_<YYYYmmdd_HHMMSS>.png`. Each line has the
form `ID <id> <image-corner> center=(<x>, <y>) side=<s> px`, followed
by a `scale: <v> px/mm` line and the PNG path. On the real table the
human run found ID 2 top-left, ID 3 top-right, ID 0 bottom-left and
ID 1 bottom-right, with marker sides from 58.1 to 59.3 px and a scale
of 1.465 px/mm.

The image corner of each ID (`top-left`, `top-right`, `bottom-left`,
`bottom-right`) is found from its center relative to the image center,
and the command fails when two markers fall into the same image
corner. The scale is the mean marker side in pixels divided by 40 mm.

## 12. Table coordinates (by hand only)

This step opens the real camera, so only the human runs it, never CI
and never an agent. `muse_qc/table_coords.py` converts table camera
pixels into table millimetres. The frame is defined in
`muse_qc/table_markers.py` as `TABLE_MARKER_LAYOUT_MM`: origin at the
centre of marker 0, +x towards marker 1, +y towards marker 2 (away
from the arm), units mm. The values were measured by hand with a tape
measure, centre to centre of the black marker squares: ID 2 to ID 3
977 mm, ID 0 to ID 2 531 mm, diagonal ID 2 to ID 1 1110 mm
(rectangle check 1112 mm). ID 0 to ID 1 could not be measured (the
arm is in the way) and is assumed from the rectangle. From the
project root:

```bash
uv run python -m muse_qc.table_coords
```

Expected result: one line per ID with centre in pixels, centre in
millimetres and side length in millimetres (expected 40 mm), then the
table position of the image centre, then the path of one grid copy
(grid lines every 100 mm over the marker rectangle, origin and axes
labelled) at `recordings/table_grid_<YYYYmmdd_HHMMSS>.png`, for
example:

```text
ID 0 center_px=(212.0, 932.0) center_mm=(-0.0, 0.0) side=37.2 mm (expected 40 mm)
ID 1 center_px=(1755.0, 928.8) center_mm=(977.0, 0.0) side=37.5 mm (expected 40 mm)
ID 2 center_px=(216.5, 98.0) center_mm=(0.0, 531.0) side=37.0 mm (expected 40 mm)
ID 3 center_px=(1755.5, 93.0) center_mm=(977.0, 531.0) side=37.8 mm (expected 40 mm)
image center px=(960.0, 540.0) mm=(473.4, 248.0)
recordings/table_grid_20261028_003926.png
```

The command fails when a marker is missing, naming the missing IDs.
The side length in mm is an independent check: it maps the four
marker corners (not the centres used for the homography) into the
table frame, so a wrong homography shows up as a wrong side length.

When `calibration/table_camera.json` exists (see section 14), the
command loads it and prints
`calibration: calibration/table_camera.json (rms <x> px)` as the
first line; otherwise it prints `calibration: none (raw pixels)`.
With a calibration, the detected marker centres and corners are
undistorted point by point before the homography is built, every
pixel-to-mm mapping undistorts the raw pixel first, and the grid is
distorted back onto the raw frame, which itself is never
undistorted. `detect_markers.py` is unchanged and always reports raw
pixels.

Known limitation without the calibration file: the lens distortion
of the table camera is not corrected. The marker sides come out at
about 37 mm instead of 40 mm, and a 25 mm cube placed by tape
measure at x = 300 mm, y = 300 mm appeared about 15 mm too far in
-x (y matched). Good enough to locate parts roughly; the arm camera
refines the position before a grasp. The example output above was
recorded without a calibration.

## 13. Calibration checkerboard (print by hand)

This step uses only the printer, so the human runs it by hand, never CI
and never an agent. `muse_qc/checkerboard.py` writes one A4 landscape
page with the calibration checkerboard (shared constants in
`muse_qc/checkerboard.py`: 10 by 7 squares of 25 mm, 9 by 6 inner
corners). From the project root:

```bash
uv run python -m muse_qc.checkerboard
```

Expected result: one printed line with the path `docs/checkerboard.svg`.
The file is part of the repository, so it can be printed straight from
there. It can be generated again with the command above.

Print the page at 100 % (actual size, no scaling to fit the page).
Then check with a ruler: the scale line must measure exactly 100 mm
and one square exactly 25 mm. Glue the page flat on cardboard so it
stays plane for the calibration.

## 14. Table camera calibration (by hand only)

This step opens the real camera, so only the human runs it, never CI
and never an agent. The arm stays powered off so it cannot move into
the frame. `muse_qc/capture_calibration.py` takes the photos and
`muse_qc/calibrate_camera.py` computes the calibration offline from
the saved PNGs. Both import no arm code and move nothing.

Photo plan: take about 16 photos of the flat board, always fully
visible with a white margin around it. Take about 8 flat photos
spread over the centre, the four corners and the edges of the image,
then about 8 tilted photos with the board leaning 20 to 40 degrees
against a box or book (different tilt directions). Move the board,
never the camera.

Capture, from the project root:

```bash
uv run python -m muse_qc.capture_calibration
```

Expected result: a terminal loop with `Enter = take photo, q =
quit`. Each photo is taken with `open_table_camera()` from
`muse_qc/camera_top.py` and checked with
`cv2.findChessboardCornersSB` (9 by 6 inner corners). Photos with a
found board print `board found` and are saved to
`recordings/calibration/calib_<timestamp>.png`, the others print
`board NOT found` and are skipped. Every saved photo prints the
running count.

Calibrate, from the project root:

```bash
uv run python -m muse_qc.calibrate_camera
```

Expected result: one line per used image (`images used: <n>`),
the RMS reprojection error in px (well below 1 px on good photos),
the distortion coefficients, and the path
`calibration/table_camera.json` with `camera_matrix`, `dist_coeffs`,
`image_size`, `rms_px`, `images_used` and `created`. The command is
refused with a clear error when fewer than 10 boards are found or
when the images differ in size. `load_calibration()`, `undistort()`,
`undistort_points()` and `distort_points()` in the same module are
reused by the table mapping; `detect_markers.py` always works on
the raw frame.

Human result on the real table: 26 images used, RMS reprojection
error 0.199 px, fx about 1518 px, principal point about (959, 542),
mild barrel distortion; `calibration/table_camera.json` written.
Expected first line of table_coords with the file present:
`calibration: calibration/table_camera.json (rms 0.199 px)`.

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
  `Arm.__init__` only stores the factory,
  `Arm.connect` only calls `sdk.connect()`, and
  `Arm.disconnect` only calls `sdk.disconnect()`.
- Not verified: the exact bytes the `motorbridge` package
  (`Controller.disable_all`, `shutdown`, `close`, `send_*`) puts on the
  bus. That package was outside this review.
