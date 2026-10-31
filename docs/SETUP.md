# Setup

One machine holds both this project and the Seeed SDK checkout.
The SDK lives outside this repository; only the dependency pointer lives here.

## Module move 2026-10-31: old command -> new command

On 2026-10-31 the flat modules of `muse_qc/` moved into subpackages
(`agent/`, `vision/`, `inspection/`, `arm/`, `gripper/`, `pick/`,
`checks/`, `bringup/`). Nothing else changed. Use the new command:

| Old command | New command |
| --- | --- |
| `python -m muse_qc.agent` | `python -m muse_qc.agent` (unchanged) |
| `python -m muse_qc.arm_calibration` | `python -m muse_qc.arm.arm_calibration` |
| `python -m muse_qc.target_pose` | `python -m muse_qc.arm.target_pose` |
| `python -m muse_qc.xy_correction` | `python -m muse_qc.arm.xy_correction` |
| `python -m muse_qc.diagnose_joints` | `python -m muse_qc.bringup.diagnose_joints` |
| `python -m muse_qc.first_move` | `python -m muse_qc.bringup.first_move` |
| `python -m muse_qc.fk_check` | `python -m muse_qc.bringup.fk_check` |
| `python -m muse_qc.hold_test` | `python -m muse_qc.bringup.hold_test` |
| `python -m muse_qc.hover_move` | `python -m muse_qc.bringup.hover_move` |
| `python -m muse_qc.model_check` | `python -m muse_qc.checks.model_check` |
| `python -m muse_qc.preflight` | `python -m muse_qc.checks.preflight` |
| `python -m muse_qc.gripper` | `python -m muse_qc.gripper.gripper` |
| `python -m muse_qc.gripper_grip` | `python -m muse_qc.gripper.gripper_grip` |
| `python -m muse_qc.gripper_motor` | `python -m muse_qc.gripper.gripper_motor` |
| `python -m muse_qc.gripper_watch` | `python -m muse_qc.gripper.gripper_watch` |
| `python -m muse_qc.count_clips` | `python -m muse_qc.inspection.count_clips` |
| `python -m muse_qc.count_retry` | `python -m muse_qc.inspection.count_retry` |
| `python -m muse_qc.inspection.report` | `python -m muse_qc.inspection.report` (unchanged) |
| `python -m muse_qc.pick_test` | `python -m muse_qc.pick.pick_test` |
| `python -m muse_qc.calibrate_camera` | `python -m muse_qc.vision.calibrate_camera` |
| `python -m muse_qc.camera_arm` | `python -m muse_qc.vision.camera_arm` |
| `python -m muse_qc.camera_top` | `python -m muse_qc.vision.camera_top` |
| `python -m muse_qc.capture_calibration` | `python -m muse_qc.vision.capture_calibration` |
| `python -m muse_qc.checkerboard` | `python -m muse_qc.vision.checkerboard` |
| `python -m muse_qc.detect_markers` | `python -m muse_qc.vision.detect_markers` |
| `python -m muse_qc.find_parts` | `python -m muse_qc.vision.find_parts` |
| `python -m muse_qc.make_markers` | `python -m muse_qc.vision.make_markers` |
| `python -m muse_qc.table_coords` | `python -m muse_qc.vision.table_coords` |
| `python -m muse_qc.vision_sam` | `python -m muse_qc.vision.vision_sam` |

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

## 7. Trying muse_qc/arm/arm.py on the real arm (by hand only)

This step touches hardware, so only the human runs it, never CI and
never an agent. `muse_qc/arm/arm.py` only reads: it cannot enable, zero or move
the arm (enable, zero, mode and send commands are unreachable through
it), but `connect()` still opens the real bus. From the project root:

```bash
uv run python -c "
from muse_qc.arm.arm import Arm
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
feedback, so `muse_qc/arm/arm.py` never reports `0.0` as an angle. Check the bus
connection and the motor power, then retry.

## 8. Finding the arm by USB ID (by hand only)

This step is run by the human only, never by an agent. The arm port is
never a fixed `/dev/ttyACM0`: `muse_qc/checks/preflight.py` finds it by USB vendor and
product ID through `muse_qc/checks/usb_devices.py` (read-only sysfs lookup, the serial
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
uv run python -m muse_qc.checks.preflight
```

Expected result: `arm-usb: OK` with the found path. With the USB cable
unplugged: `arm-usb: FAIL` (`arm not found by USB ID 2e88:4603`), and the
port checks report `skipped: no arm port` instead of probing.

`muse_qc/checks/preflight.py` now also checks that the SDK opens the same device:
`sdk-port` compares the SDK channel from `muse_qc/arm/sdk_config.py`
(`config/rebotarm.yaml` via `hardware_yaml`) with the arm port found by
USB ID. Expected: `sdk-port: OK`. It FAILs when the channel is
unreadable, is not a `/dev` path (for example `can0`), or points to a
different device; without an arm port it reports `skipped: no arm port`.

Never write the device serial number into the repository.

## 9. Table camera test (by hand only)

This step opens the real camera, so only the human runs it, never CI
and never an agent. `muse_qc/vision/camera_top.py` finds the table camera by
USB vendor and product ID through `muse_qc/checks/usb_devices.py`
(read-only sysfs lookup under `/sys/class/video4linux`, capture nodes
only), opens exactly one matching device with the V4L2 backend, requests
MJPG at 1920x1080, discards 45 warm-up frames (a short settle time,
letting the camera settle after the settings are applied), then reads
the returned frame and saves it. It
imports no arm code and moves nothing. From the project root:

```bash
uv run python -m muse_qc.vision.camera_top
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
and never an agent. `muse_qc/vision/make_markers.py` writes one A4 page with
the four ArUco table markers (IDs 0 to 3, shared constants in
`muse_qc/vision/table_markers.py`). From the project root:

```bash
uv run python -m muse_qc.vision.make_markers
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
and never an agent. `muse_qc/vision/detect_markers.py` takes one frame with
`open_table_camera()` from `muse_qc/vision/camera_top.py`, finds the four
ArUco table markers (IDs 0 to 3, shared constants in
`muse_qc/vision/table_markers.py`) with `cv2.aruco.ArucoDetector`, and
requires each ID exactly once: a missing marker raises naming the
missing IDs, a marker seen twice raises naming the duplicated IDs.
Detected IDs outside 0 to 3 are ignored and reported under
`ignored_ids`. From the project root:

```bash
uv run python -m muse_qc.vision.detect_markers
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
and never an agent. `muse_qc/vision/table_coords.py` converts table camera
pixels into table millimeters. The frame is defined in
`muse_qc/vision/table_markers.py` as `TABLE_MARKER_LAYOUT_MM`: origin at the
center of marker 0, +x towards the center of marker 1, units mm. The
four points are a least-squares fit (residuals under 0.5 mm) to all
six hand-measured center-to-center distances of the black marker
squares: ID0-ID1 982 mm, ID2-ID3 977 mm, ID0-ID2 531 mm,
ID1-ID3 530 mm, diagonals ID0-ID3 1117 mm and ID1-ID2 1110 mm. The
markers form a slightly skewed quadrilateral, not a rectangle. From
the project root:

```bash
uv run python -m muse_qc.vision.table_coords
```

Expected result: one line per ID with center in pixels, center in
millimeters and side length in millimeters (expected 40 mm), then the
table position of the image center, then the path of one grid copy
(grid lines every 100 mm over the marker quadrilateral, origin and
axes labeled) at `recordings/table_grid_<YYYYmmdd_HHMMSS>.png`.
Real human run with the calibration and the fitted quadrilateral
layout:

```text
calibration: calibration/table_camera.json (rms 0.199 px)
ID 0 center_px=(212.2, 931.5) center_mm=(-0.0, 0.0) side=39.5 mm (expected 40 mm)
ID 1 center_px=(1754.5, 928.8) center_mm=(982.0, 0.0) side=39.6 mm (expected 40 mm)
ID 2 center_px=(216.8, 98.0) center_mm=(6.9, 530.8) side=39.3 mm (expected 40 mm)
ID 3 center_px=(1755.5, 93.0) center_mm=(983.6, 529.9) side=39.9 mm (expected 40 mm)
image center px=(960.0, 540.0) mm=(478.5, 248.4)
recordings/table_grid_20261028_084104.png
```

The command fails when a marker is missing, naming the missing IDs.
The side length in mm is an independent check: it maps the four
marker corners (not the centers used for the homography) into the
table frame, so a wrong homography shows up as a wrong side length.

When `calibration/table_camera.json` exists (see section 14), the
command loads it and prints
`calibration: calibration/table_camera.json (rms <x> px)` as the
first line; otherwise it prints `calibration: none (raw pixels)`.
With a calibration, the detected marker centers and corners are
undistorted point by point before the homography is built, every
pixel-to-mm mapping undistorts the raw pixel first, and the grid is
distorted back onto the raw frame, which itself is never
undistorted. `detect_markers.py` is unchanged and always reports raw
pixels.

Known limitation without the calibration file: the lens distortion
of the table camera is not corrected. The marker sides come out at
about 37 mm instead of 40 mm, and a 25 mm cube placed by tape
measure at x = 300 mm, y = 300 mm appeared about 15 mm too far in
-x (y matched). With the calibration the cube re-read from the
grid image drawn with the previous rectangle layout sat at about
x = 291 mm, y = 311 mm. With the calibration and the fitted layout
the same unchanged cube reads about x = 297 mm, y = 312 mm, so
3 mm off in x and 12 mm in y; the remaining y offset is probably
the tape-measure placement and will be checked with the arm. Good
enough to locate parts roughly; the arm camera refines the position
before a grasp.

## 13. Calibration checkerboard (print by hand)

This step uses only the printer, so the human runs it by hand, never CI
and never an agent. `muse_qc/vision/checkerboard.py` writes one A4 landscape
page with the calibration checkerboard (shared constants in
`muse_qc/vision/checkerboard.py`: 10 by 7 squares of 25 mm, 9 by 6 inner
corners). From the project root:

```bash
uv run python -m muse_qc.vision.checkerboard
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
the frame. `muse_qc/vision/capture_calibration.py` takes the photos and
`muse_qc/vision/calibrate_camera.py` computes the calibration offline from
the saved PNGs. Both import no arm code and move nothing.

Photo plan: take about 16 photos of the flat board, always fully
visible with a white margin around it. Take about 8 flat photos
spread over the center, the four corners and the edges of the image,
then about 8 tilted photos with the board leaning 20 to 40 degrees
against a box or book (different tilt directions). Move the board,
never the camera.

Capture, from the project root:

```bash
uv run python -m muse_qc.vision.capture_calibration
```

Expected result: a terminal loop with `Enter = take photo, q =
quit`. Each photo is taken with `open_table_camera()` from
`muse_qc/vision/camera_top.py` and checked with
`cv2.findChessboardCornersSB` (9 by 6 inner corners). Photos with a
found board print `board found` and are saved to
`recordings/calibration/calib_<timestamp>.png`, the others print
`board NOT found` and are skipped. Every saved photo prints the
running count.

Calibrate, from the project root:

```bash
uv run python -m muse_qc.vision.calibrate_camera
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

## 15. Model API connection check (by hand only)

This step contacts the Meta Model API over the network, so only the
human runs it, never CI and never an agent. `muse_qc/checks/model_check.py`
reads `MODEL_API_KEY` from `.env` or the environment (the key is never
printed, not even partly), calls `GET /v1/models` once through the
`openai` SDK with the base URL `https://api.meta.ai/v1`, prints the
number of models and one `OK`/`MISSING` line each for `muse-spark-1.3`
and `sam-3.1`. The exit code is 0 only when both are listed. From the
project root:

```bash
uv run python -m muse_qc.checks.model_check
```

Real human run with a valid key (recorded output, never edited):

```text
models: 9
muse-spark-1.3: OK
sam-3.1: OK
```

Exit code 0. With an invalid key the same run prints `key rejected`
with exit 1 (the API answers HTTP 401 invalid_api_key).

With a missing key the command exits nonzero with a message that
`MODEL_API_KEY` is missing. With a wrong key it prints `key rejected`.
On network errors it prints a short reason. None of these cases prints
a stack trace or the key.

## 16. SAM 3.1 box detection (by hand only)

This step opens the real table camera and contacts the Meta Model API
over the network, so only the human runs it, never CI and never an
agent. `muse_qc/vision/vision_sam.py` takes one frame with
`open_table_camera()` from `muse_qc/vision/camera_top.py` (or a saved image
with `--image`), sends it with a noun phrase to SAM 3.1 through the
`openai` SDK in one `POST /v1/responses` call, parses the box records
from the output text (masks are ignored) and draws the boxes with IDs
onto a copy of the image. The key is read with
`muse_qc/checks/model_check.py` and never printed. From the project root:

```bash
uv run python -m muse_qc.vision.vision_sam "small part"
uv run python -m muse_qc.vision.vision_sam "small part" --image path.png
```

Result format: one line per box with id, box corners, center in
pixels and confidence, then the count, then the path of the annotated
copy at `recordings/sam_<YYYYmmdd_HHMMSS>.png`. Before the box lines,
one `table mapping:` line reports `calibrated`, `raw pixels` (no
calibration file) or `unavailable (<reason>)`, and each box line ends
with `table_mm=(x, y)` in millimeters (the box center mapped onto the
table frame on the same frame sent to SAM) when the mapping works.

Real human run of `uv run python -m muse_qc.vision.vision_sam "small part"`
on the live table camera with two connectors and one white 25 mm cube
on the table, all four table markers visible (recorded output, never
edited):

```text
table mapping: calibrated
id 0 box=(638, 417, 690, 460) center=(664.0, 438.5) confidence=0.52 table_mm=(294.8, 311.9)
id 1 box=(1136, 377, 1176, 413) center=(1156.0, 395.0) confidence=0.65 table_mm=(600.1, 337.8)
id 2 box=(1168, 188, 1209, 227) center=(1188.5, 207.5) confidence=0.68 table_mm=(620.0, 452.8)
boxes: 3
recordings/sam_20261028_105815.png
```

id 0 is the white cube, id 1 and id 2 are the two connectors (checked
by eye on the annotated image). In this run "small part" also matched
the cube, so the phrase finds any small object; the arm camera check
decides what a part really is.

An empty answer means zero matches (`boxes: 0`), not an error. HTTP
400 prints `prompt rejected` with the API message, 401/403 prints
`key rejected`, network errors print a short reason; none of these
prints a stack trace or the key.

## 17. Forward kinematics check (by hand only)

This step opens the real arm bus for reading, so only the human
runs it, never CI and never an agent. `muse_qc/bringup/fk_check.py` reads
the six joint angles once through `muse_qc/arm/arm.py` (read only: it
never enables, zeroes or sends anything), computes the `end_link`
pose with the Seeed SDK kinematics (meters and radians in the SDK,
printed as mm and degrees) and projects the finger-tip position
30 mm along the tool axis (link6 to end_link). The 30 mm value
(`TIP_OFFSET_MM`) is an estimate from a hand measurement (185 mm
from the joint 6 mounting face to the finger tips, minus the
155.39 mm URDF `end_joint` offset) and must be verified on the arm.
From the project root:

```bash
uv run python -m muse_qc.bringup.fk_check
```

Expected result: one line per joint with the angle in degrees, one
`end_link:` line with x, y, z in mm and roll, pitch, yaw in
degrees, and one `tip:` line with x, y, z in mm.

Recorded output of the FIRST version (recorded output is never
edited):

```text
joint1: -3.33 deg
joint2: +1.58 deg
joint3: -1.45 deg
joint4: +2.72 deg
joint5: +0.91 deg
joint6: -3.97 deg
end_link: x=+256.6 y=-17.9 z=+198.6 mm roll=-3.97 pitch=-0.32 yaw=-4.24 deg
tip: x=+256.6 y=-15.8 z=+228.6 mm (offset 30 mm along end_link z)
```

That run showed the wrong tip direction (30 mm straight up while
the gripper points forward) and led to this fix: the tip is now
projected along the tool axis from link6 to end_link instead of
along the end_link z axis. Recorded output of the fixed version (recorded output is never edited):

```text
joint1: -3.33 deg
joint2: +1.58 deg
joint3: -1.45 deg
joint4: +2.72 deg
joint5: +0.91 deg
joint6: -3.97 deg
end_link: x=+256.6 y=-17.9 z=+198.6 mm roll=-3.97 pitch=-0.32 yaw=-4.24 deg
tip: x=+286.5 y=-20.1 z=+198.8 mm (offset 30 mm along tool axis)
```

## 18. First arm motion (by hand only)

Program: `muse_qc/bringup/first_move.py` with the plan in
`muse_qc/bringup/first_move_plan.py`, checked by
`tests/test_first_move.py` (fake arm only). Default is a dry run:
connect, read the six joints, set J6_REF from the fresh joint 6
reading, then check every waypoint through `safety.check_target`
and print one line per waypoint (joints in deg, tip x/y/z in mm,
safety result). The dry run never enables the motors and never
sends anything. With `--execute` the same validation runs first,
then the human types `yes`; only then the arm ramps from the
current reading to home (all joints 0.0 deg), moves joint1 to
+5.0 deg and back, and holds for 2 s. The ramp is linear in joint
space at 5.0 deg/s and 100 Hz (0.05 deg per step, loop rate set
explicitly because the SDK default is 500 Hz), the motor speed
caps run at 10 % of the SDK config values on every send, and
every control cycle re-checks the next step against a fresh
reading. On a refusal or Ctrl+C the program keeps commanding
the last commanded position (never a live reading, which can
be 0.0 on missing feedback), then stops the loop and
disconnects (motors unpowered). The FK model is loaded once.
After the home hold the plan appends a settle segment: it ramps
to the rest reading clamped into its URDF limits and holds it
for 1 s. The arm is brought back close to its rest reading
(joints outside their URDF limits stay at the limit), so it
barely drops when the motors are disabled. Cycle timing: the dry run
prints a `cycle work (dry run):` line with mean, p95 and max of the
per-waypoint work (joint read, tip FK and safety check), and
`--execute` prints `cycle work:` plus `cycle period:` (time between
callback starts with the loop-rate target). Both are pure measurements
and change neither what is sent nor when.

Recorded settle run (2026-10-28, arm power supply on, vlim 10 %, recorded at 50 Hz before the switch to 100 Hz):

```text
rest joints deg: ['-2.26', '+0.12', '-0.08', '+2.31', '+0.86', '-0.08']
plan: 298 waypoints at 50 Hz
wp 001/298 [home] joints=[-2.17 +0.12 -0.07 +2.21 +0.83 -0.07] tip=[+289.5 -14.0 +181.7] ok
...
wp 024/298 [home] joints=[+0.00 +0.00 +0.00 +0.00 +0.00 +0.00] tip=[+290.3 +0.0 +191.7] ok
...
wp 074/298 [out] joints=[+5.00 +0.00 +0.00 +0.00 +0.00 +0.00] tip=[+289.2 +25.3 +191.7] ok
...
wp 124/298 [back] joints=[+0.00 +0.00 +0.00 +0.00 +0.00 +0.00] tip=[+290.3 +0.0 +191.7] ok
wp 125/298 [hold] joints=[+0.00 +0.00 +0.00 +0.00 +0.00 +0.00] tip=[+290.3 +0.0 +191.7] ok
...
wp 224/298 [hold] joints=[+0.00 +0.00 +0.00 +0.00 +0.00 +0.00] tip=[+290.3 +0.0 +191.7] ok
wp 225/298 [settle] joints=[-0.09 +0.00 -0.00 +0.10 +0.04 -0.00] tip=[+290.3 -0.6 +191.2] ok
...
wp 248/298 [settle] joints=[-2.26 +0.00 -0.08 +2.31 +0.86 -0.08] tip=[+289.5 -14.7 +180.7] ok
...
wp 298/298 [settle] joints=[-2.26 +0.00 -0.08 +2.31 +0.86 -0.08] tip=[+289.5 -14.7 +180.7] ok
SDK vlim: ['5.0', '5.0', '5.0', '3.0', '3.0', '3.0'] -> 10 %: ['0.50', '0.50', '0.50', '0.30', '0.30', '0.30']
Type yes to move the arm: yes
motion finished, arm holds home
stopping: the motors will be disabled now
```

Note: the end message was changed to "arm holds the settle pose"
after this run, because the plan ends in the settle pose.

Recorded 100 Hz run (2026-10-28, arm power supply on, vlim 10 %):

```text
cycle work (dry run): mean 1.20 ms, p95 1.25 ms, max 8.07 ms over 590 waypoints
cycle work: mean 2.04 ms, p95 2.10 ms, max 2.21 ms over 590 cycles
cycle period: mean 10.09 ms, p95 10.16 ms, max 10.20 ms (target 10.00 ms)
motion finished, arm holds the settle pose
stopping: the motors will be disabled now
```

`RebotArmEndPose`, zeroing and the home routine are never used.

Safety steps for the human:

1. Hand on the power supply switch (emergency stop), nothing on
   the table near the arm.
2. First the dry run below. It must print `ok` for every
   waypoint. Known finding: the recorded rest position has
   joint2 at +1.58 deg, just above the SDK URDF upper limit
   (0.0 deg), because the arm rests on its mechanical stop.
   `safety.check_target` therefore allows an out-of-range target
   only when the current reading is outside the same limit by at
   most 3.0 deg (`LIMIT_RECOVERY_DEG`) and the target moves back
   toward the range, never further out; anything else outside
   the limits is refused with `joint-limit` and `--execute`
   aborts without moving. Do not move until the rest reading
   and the plan pass the dry run.
3. Only then `--execute`, confirm with `yes`, watch the arm.

```bash
uv run python -m muse_qc.bringup.first_move
uv run python -m muse_qc.bringup.first_move --execute
```

## 19. Arm camera test (by hand only)

This step opens the real arm camera, so only the human runs it, never CI
and never an agent. `muse_qc/vision/camera_arm.py` finds the color stream of
the Orbbec Gemini 305 on the gripper by USB vendor and product ID
(2bc5:0840) plus USB interface 4 (`find_arm_color_node` in the same
module: the interface directory behind the `device` link must carry
`bInterfaceNumber` 04 and the node `index` must be 0), never by fixed
`/dev` number. The node is opened with the V4L2 backend as YUYV
640x480, warmed up with 15 reads and retried up to 3 times (the first
open sometimes gives no frame), then one frame is saved unrotated:
the image is upright. It imports no arm code and moves nothing.
The Orbbec SDK is not used and is not a dependency. From the project
root:

```bash
uv run python -m muse_qc.vision.camera_arm
```

Expected result: one new file `recordings/armcam_<YYYYmmdd_HHMMSS>.png`
at 640x480 plus printed lines with the node, the attempt number and
the frame shape, for example `node: /dev/video6`. Exit codes: 0 ok,
1 camera not found, 2 no frame, 3 write failed.

Images in `recordings/` are never committed (`recordings/` is
git-ignored). Recorded output (2026-10-28, arm power supply off):

```text
node: /dev/video6
attempt 1 ok: True
frame shape: (480, 640, 3)
saved recordings/armcam_20261028_133753.png
exit 0
```

The last line is the shell exit code.

## 20. Count levers on an arm camera image (by hand only)

This step contacts the Meta Model API, so only the human runs it,
never CI and never an agent. `muse_qc/inspection/count_clips.py` works in two
steps on a saved arm camera image (for example from section 19).
First it sends the full image with `PART_PROMPT` ("small part") to
SAM 3.1 through the `muse_qc/vision/vision_sam.py` request code, ignores
boxes in the gripper zone (box center y at or above
`GRIPPER_ZONE_Y` = 380 px; the gripper is fixed in the arm camera
image) and picks the remaining box with the highest confidence at
or above `MIN_CONFIDENCE` (0.5). Then it crops that box plus
`CROP_MARGIN_PX` = 15 on each side (clamped to the image), enlarges
the crop `CROP_SCALE` = 8 times with `cv2.INTER_CUBIC`, and sends
the crop with `LEVER_PROMPT` ("brown bar") to SAM 3.1 again. Lever
boxes at or above confidence 0.5 are kept, overlapping duplicates
are merged (IoU above 0.5 keeps the higher confidence), and a box
is kept only when it reaches `MIN_BAR_PART_FRAC` = 0.35 of the
enlarged part height (without the crop margin) and
`MIN_BAR_TALLEST_FRAC` = 0.6 of the tallest confident bar;
shorter boxes are dropped as torn-out remnants (printed as
`dropped (too short): ...`). The part-height rule replaces the old
crop-height rule, whose margin made the limit depend on the part
size in the image. The program
prints `part: box=(...) confidence=...`, one line per kept lever
box with its confidence, then `levers: N` and
`verdict: good|bad` (good means exactly `EXPECTED_LEVERS` = 3
levers), and saves two annotated copies under one timestamp:
`recordings/count_<YYYYmmdd_HHMMSS>.png` (full image with the part
box and `levers: N`) and
`recordings/count_<YYYYmmdd_HHMMSS>_crop.png` (enlarged crop with
the lever boxes), checking both write results. With `--image`,
`muse_qc/vision/vision_sam.py` skips the table mapping and prints
`table mapping: skipped (image file)`. The API key comes only from
the existing `.env` loading code and is never printed. From the
project root:

```bash
uv run python -m muse_qc.inspection.count_clips --image recordings/armcam_test.png
uv run python -m muse_qc.inspection.count_clips --image recordings/armcam_test.png --prompt "brown bar"
```

`--prompt` overrides only the lever prompt; the part prompt is
always `PART_PROMPT`. With `--expect-target` only the part near
`EXPECTED_PART_PX` = (342, 290) is accepted (up to
`MAX_PART_OFFSET_PX` = 60 px away), as the gripper inspection
does, so a saved gripper image can be re-checked the same way:

```bash
uv run python -m muse_qc.inspection.count_clips --image recordings/armcam_test.png --expect-target
```

All values above are constants at the top of
`muse_qc/inspection/count_clips.py`.

Exit codes: 0 done, 1 image not readable (or the crop or an
annotated copy cannot be written), 2 API error (missing key or
failed request), 3 no part found (`part: not found`).

Images in `recordings/` are never committed (`recordings/` is
git-ignored). Recorded output (2026-10-28, arm power supply off,
test pair good/bad):

```text
=== 145156
part: box=(317, 187, 352, 233) confidence=0.76
id 1 box=(158, 206, 210, 457) center=(184.0, 331.5) confidence=0.70
id 2 box=(237, 206, 289, 457) center=(263.0, 331.5) confidence=0.70
id 0 box=(313, 213, 363, 464) center=(338.0, 338.5) confidence=0.68
levers: 3
verdict: good
recordings/count_20261028_164947.png
recordings/count_20261028_164947_crop.png
=== 145329
part: box=(319, 186, 354, 232) confidence=0.75
dropped (too short): id 1 box=(323, 206, 364, 296) confidence=0.59
id 2 box=(241, 201, 290, 457) center=(265.5, 329.0) confidence=0.77
id 0 box=(162, 205, 212, 458) center=(187.0, 331.5) confidence=0.75
levers: 2
verdict: bad
recordings/count_20261028_164952.png
recordings/count_20261028_164952_crop.png
```

Every successful run also writes
`recordings/count_<YYYYmmdd_HHMMSS>.json` next to the two PNGs
and prints its path as the last line. The JSON record holds the
timestamp, the source image path, the full and crop image paths,
the part box and confidence, the lever prompt, the kept lever
boxes with confidence, the dropped boxes, the lever count, the
verdict and the expected lever count. It is written by
`muse_qc/inspection/record.py` (with the drawing helpers in
`muse_qc/inspection/draw.py`).

## 21. Inspection report (offline)

This step reads only saved files, so it runs anywhere without
hardware, camera, network or API key. `muse_qc/inspection/report.py`
reads all `recordings/count_*.json` records and writes one
self-contained HTML page (inline CSS only, no external URLs, no
JavaScript; the crop and full images are referenced by relative
path). The page header shows the title, the generation time and
the totals (parts inspected, good, bad); each record gets one
card, newest first, with the time, the crop thumbnail, a full
image link, levers found versus expected, a green GOOD or red BAD
badge, and the dropped remnant boxes when there are any. With no
records yet the page says "no inspections yet". From the project
root:

```bash
uv run python -m muse_qc.inspection.report --recordings recordings --out recordings/report.html
```

Exit codes: 0 report written, 1 write failed. The command prints
the output path.

Recorded output:

```text
recordings/report.html
```

The page showed "parts inspected: 2, good: 1, bad: 1" with the BAD card first and the GOOD card second; the times on the page are UTC.

## 22. Target pose dry run (by hand only)

This step turns a part position in table millimeters (as printed by
`muse_qc/vision/vision_sam.py`, table frame of `muse_qc/vision/table_coords.py`:
origin at the center of marker 0, +x towards marker 1, +y towards
marker 2 away from the arm) into an arm pose with the gripper tip
straight above the part at a hover height. `muse_qc/arm/target_pose.py`
converts the table target to the arm base frame (joint 1 axis at
table (495.9, 22.7) mm from the human tape measurement of
2026-10-28), places `end_link` 30 mm above the tip with the tool
axis pointing straight down, solves the SDK IK
(`compute_ik` from `reBotArm_control_py.kinematics`) seeded with
the start joints, verifies the FK tip against the target (2 mm),
then ramps from the start joints to the solved joints at the
`first_move` speed (5 deg/s at 100 Hz) plus a 2 s hold and checks
every waypoint with `safety.check_target`. Dry run only: nothing
is enabled, nothing is sent, and the module offers no motion flag.
From the project root, offline first (needs no hardware):

```bash
uv run python -m muse_qc.arm.target_pose --table-mm 495.9 322.7 --start home
```

Then with the arm power supply on (still no motion, reads the
current joints once through the read-only `muse_qc/arm/arm.py`):

```bash
uv run python -m muse_qc.arm.target_pose --table-mm 495.9 322.7 --start arm
```

Exit codes: 0 plan ok, 1 unreachable or a waypoint rejected,
2 arm read failed. The output shows the table target, the base
target, the hover height, the IK joints in degrees, the FK tip in
base and table mm, the error to the target in mm, the number of
waypoints, and either "all waypoints ok" or the first rejected
waypoint with the safety reason, always ending with
"dry run: no motion". The forward kinematics check showed the real
tip about 13 mm lower than computed (DEVLOG 2026-10-28), so a
100 mm hover is about 87 mm in reality.

Recorded output (--start arm after the check_plan fix, 2026-10-28):

```text
table target: x=+495.90 y=+322.70 mm
base target: x=+300.00 y=-0.00 mm
hover: +100.0 mm
start joints deg: -1.89 +0.69 -0.23 +2.55 +1.30 -0.08
ik joints deg: -0.00 -97.43 -82.51 +75.08 +0.00 +0.01
fk tip base mm: x=+300.00 y=-0.00 z=+99.99
fk tip table mm: x=+495.90 y=+322.70 z=+99.99
error to target: 0.01 mm
waypoints: 2163
all waypoints ok
dry run: no motion
exit 0
```

## 23. Hover move (by hand only)

Program: `muse_qc/bringup/hover_move.py`, checked by
`tests/test_hover_move.py` (fake arm only). It moves the gripper
tip from the current rest pose to the hover pose above a table
target (the same pose `muse_qc/arm/target_pose.py` already solves and
checks: table frame of `muse_qc/vision/table_coords.py`, IK seeded with
the start joints), holds it for 2 s, ramps back to the rest
reading clamped into its URDF limits, holds it for 1 s, and only
then lets `first_move` disable the motors. The arm is never
disabled while it is away from the rest pose. Default is a dry
run (no enable, no send); with `--execute` the human confirms
with `yes`, the motors run at 10 % of the SDK speed caps, and
every control cycle re-checks the next step. A failed solve
prints the reason and moves nothing. Targets below 80 mm hover
are refused in this module (the real tip sits lower than FK, see docs/DEVLOG.md 2026-10-28). Option `--hold-s` sets the hover hold time in seconds
(default 2 s, up to 30 s), used for hand measurements during the hold.

Safety steps for the human:

1. Hand on the power supply switch (emergency stop), nothing on
   the table near the arm.
2. Watch all links during the motion: `safety.py` checks only
   the tool tip, not link collisions.
3. On Ctrl+C the program holds the last commanded position and
   then disables; the arm can drop.

```bash
uv run python -m muse_qc.bringup.hover_move --table-mm 495.9 322.7
uv run python -m muse_qc.bringup.hover_move --table-mm 495.9 322.7 --execute
```

Recorded output (--execute, 2026-10-28, arm power supply on, hand on the power supply switch, nothing on the table; per-waypoint lines cut, recorded output is never edited):

```text
rest joints deg: ['-1.89', '+0.69', '-0.23', '+2.55', '+1.30', '-0.08']
table target: x=+495.90 y=+322.70 mm
base target: x=+300.00 y=-0.00 mm
hover: +100.0 mm
start joints deg: -1.89 +0.69 -0.23 +2.55 +1.30 -0.08
ik joints deg: -0.00 -97.43 -82.51 +75.08 +0.00 +0.01
fk tip base mm: x=+300.00 y=-0.00 z=+99.99
fk tip table mm: x=+495.90 y=+322.70 z=+99.99
error to target: 0.01 mm
plan: 4212 waypoints at 100 Hz
...
cycle work (dry run): mean 1.17 ms, p95 1.23 ms, max 1.33 ms over 4212 waypoints
SDK vlim: ['5.0', '5.0', '5.0', '3.0', '3.0', '3.0'] -> 10 %: ['0.50', '0.50', '0.50', '0.30', '0.30', '0.30']
Type yes to move the arm: cycle work: mean 1.95 ms, p95 2.03 ms, max 2.19 ms over 4212 cycles
cycle period: mean 10.12 ms, p95 10.17 ms, max 10.23 ms (target 10.00 ms)
motion finished, arm holds the settle pose
stopping: the motors will be disabled now
```

In this first run joint 4 moved in small steps and joint 5 jittered slightly; this is open and not yet explained.

In a second run the human measured the tip at about 65 mm above the table for an FK hover of 100 mm; until this is explained, no target below 100 mm FK hover. Further measurements on 2026-10-28 located the error between joint 3 and joint 4 (about 5 deg at the hover pose); see docs/DEVLOG.md. The correction below fixes it. After the motion, `--execute` prints three `hold ...` lines showing the commanded hold joints, the measured hold joints and the measured tip position in mm.

Arm tip correction (`muse_qc/arm/arm_calibration.py`, checked by `tests/test_arm_calibration.py`): from the human hold measurements of 2026-10-28 (docs/DEVLOG.md) the code corrects three small errors, so a planned target lands where the real tip goes. The base sits 5 mm lower (`BASE_Z_MM = -5.0`), the gripper reaches 190 mm from the end of motor 6 (tip offset beyond end_link derived from the model, 155.39 mm), and joint 3 gets `true_q3 = reported_q3 + k * q3` with `k = -0.0556` (ratio form, fit RMS 5.45 mm; load form RMS 6.76 mm). `first_move_plan.real_tip_m` returns this corrected tip, so every `safety.check_target` sees the real height above the table. `target_pose.solve_hover` iterates the existing IK (`solve_corrected`: solve nominal, shift by the corrected error, stop below 0.5 mm or after 10 iterations), and `target_pose.run` plus `hover_move` print one extra `corrected tip table mm: ... (iterations n)` line. The 80 mm hover floor in `hover_move` stays. On 2026-10-28 the human verified the correction at (495.9, 322.7) and (495.9, 242.7) at 100 mm: both measured 100 mm (docs/DEVLOG.md). Heights below 100 mm are not verified yet. The nominal 150 mm hover at (495.9, 322.7) is unreachable once corrected (joint 4 limit).

## 24. Gripper touch sensors (by hand only)

The gripper is pre-existing hardware: Fin-Ray fingers on the reBot
gripper jaws with four magnet-plus-magnetometer sensors (L1, L2 left,
R1, R2 right), read by a Raspberry Pi Pico over USB serial. Only the
reader (`muse_qc/gripper/gripper.py`, checked by `tests/test_gripper.py`) is
new in this hackathon; hardware and firmware are not part of it. No
firmware code is used, only the documented serial line protocol, so
this MIT repository stays clear of the gripper project's GPL-3.0 code.

Run (human only, moves nothing; the tool never commands the arm;
the reader is `muse_qc/gripper/gripper.py`, the watch CLI moved to
`muse_qc/gripper/gripper_watch.py` so the reader stays at or below 300 lines):

```bash
uv run python -m muse_qc.gripper.gripper --watch 10
```

What happens: the Pico is matched by USB vendor ID 2e8a (never by
fixed /dev number), the firmware version is asked (`v`, expected 2)
and printed as `firmware: N` (only a `# firmware <N>` info line
counts; `N` inside data lines is ignored), then the tool prints "do
not touch the fingers", waits 1 s, zeroes the sensors (`n`),
switches to fast mode (`f`, about 50 lines/s) and watches for
`--watch` seconds (1 to 120, default 10). Every change of the contact
set prints one line immediately (one line is read and handled at a
time) together with what `safety.check_gripper` says for phase "open"
(informative only; that phase never latches STOP, and the tool never
clears a latched STOP itself). Ctrl+C prints a partial summary. The
summary labels the maxima "as sent by the firmware": raw-value mode
sends thousands of uT, deviation mode tens of uT. After `n` and `f`
the tool probes the next readings for up to 0.5 s; when the median
magnitude over all four sensors is above `RAW_MODE_UT` = 1000 the
firmware still streams raw values, so it sends `a` once (never twice:
`a` may be a toggle), waits 0.3 s and probes again, printing `output
mode: deviation` or `output mode: raw (a sent, still raw)`. Exit
codes: 0 ok, 1 port cannot be opened, 2 no Pico found or bad `--watch`
value.

Recorded output (second human run of 2026-10-28, shortened as marked;
see docs/DEVLOG.md). The change lines here all show +30.0s because of
the bug fixed right after these runs (lines were only evaluated at the
end); current code prints each change with its real time stamp.

```text
pico: /dev/ttyACM1
firmware reply: [about 45 raw data lines omitted] / # firmware 2 / [raw data lines omitted] (expected 2)
do not touch the fingers
start: contacts -
+30.0s contacts: L1 L2 (safety open: unexpected-contact)
+30.0s contacts: L1 L2 R2 (safety open: unexpected-contact)
+30.0s contacts: L1 L2 R1 R2 (safety open: unexpected-contact)
[20 more change lines, all +30.0s, omitted]
summary: lines=1235 data=1229 ignored=6 changes=23
events: L1=3 L2=4 R1=4 R2=5
max magnitude uT: L1=7782.0 L2=10365.0 R1=8277.0 R2=7601.0
```

## 25. Gripper motor direction test (by hand only)

This step moves the gripper motor, so only the human runs it, never CI
and never an agent. `muse_qc/gripper/gripper_motor.py` (checked by
`tests/test_gripper_motor.py`, fake SDK only) needs `--delta-deg D`
with D between -20 and +20 deg and not 0; anything else is an argparse
error (exit 2) before anything connects. No gripper-project code is
used, only the SDK motor protocol.

Dry run first (connects, reads, moves nothing):

```bash
uv run python -m muse_qc.gripper.gripper_motor --delta-deg 10
```

It prints the six arm joints and the gripper reading through
`muse_qc/arm/arm.py`, the plan (ramp out at 5 deg/s, hold 2 s, ramp back,
hold 1 s at 100 Hz) and the expectation: 0 deg = closed, more negative
= more open, about 0.32 mm total jaw opening per motor degree, so
+10 deg should close the jaws by about 3 mm. Then, with the hand on
the power supply switch and nothing between the jaws:

```bash
uv run python -m muse_qc.gripper.gripper_motor --delta-deg 10 --execute
```

After the human types `yes`, the arm and gripper groups switch to
`mode_pos_vel` and everything is enabled at once (the SDK
`JointGroup.enable()` calls the vendor controller's `enable_all()`,
so there is no gripper-only enable). The control loop then commands
the arm rest reading clamped into its URDF limits every cycle (joint 2
often rests about 1 deg beyond its limit): the arm rests on its
support and must not move. That hold pose is checked once with
`safety.check_target` first (same speed fraction as `first_move`),
and a refusal aborts without moving. The gripper runs at 10 % of its
config vlim (3.0 rad/s), the arm at 10 % of its SDK caps. At the end
of the first hold the tool prints `gripper commanded deg` and
`gripper measured deg`, at the end `gripper back deg`; then it stops
the loop and disables. Ctrl+C or any error stops the loop, disables
and prints what happened.

The human notes whether the jaws open or close and roughly by how many
mm. If +10 deg opens the jaws instead of closing them by about 3 mm,
the sign convention is the opposite of the URDF expectation and the
plan sign must be flipped before any grasp code is built on it.

## 26. First grip with touch feedback (by hand only)

This step closes the jaws onto a part the human holds, so only the
human runs it, never CI and never an agent. `muse_qc/gripper/gripper_grip.py`
(checked by `tests/test_gripper_grip.py`, fake SDK and fake serial
only; the loop lives in the shared `muse_qc/gripper/gripper_stream.py`, which
also hosts the `GripperStream` moved over from
`muse_qc/gripper/gripper_motor.py` so that module stays at or below 300
lines) needs `--open-mm W` with W between 20 and 50 mm (default 40),
`--squeeze-deg S` with S between 0 and 5 deg (default 2), and
`--speed-deg-s V` with V between 2 and 15 deg/s (default 15);
anything else is an argparse error (exit 2) before anything connects.
The default 15 deg/s stays below the gripper vlim (10 % of 3.0 rad/s,
about 17.2 deg/s), which is unchanged.
No gripper-project code is used, only the serial line protocol and the
SDK motor protocol.

Scale from the human ruler readings of 2026-10-28 (power supply off,
jaws moved by hand, tip gap with a ruler): +2.79 deg = closed (0 mm),
-4.97 deg = 5 mm, -102.63 deg = 32 mm, -167.13 deg = 53 mm, giving
`width_mm = 0.31 * (2.8 - deg)` with `CLOSED_DEG` = 2.8 and
`MM_PER_DEG` = 0.31 (the 5 mm point sits off the line, soft Fin-Ray
tips). Negative = open. The motor angle wraps at +-180 deg: fully
open by hand (98 mm) reads +33.21 deg, so +33 means nearly fully OPEN,
not closed. Every commanded target must stay inside `[SAFE_MIN_DEG,
SAFE_MAX_DEG]` = [-175.0, 5.8]; a start reading outside refuses
before enabling with "gripper reading outside the safe range (wrapped
or past closed): close the jaws by hand to about 30 mm with the power
off and run again".

Dry run first (connects, reads arm and gripper, opens the touch
sensor, moves nothing):

```bash
uv run python -m muse_qc.gripper.gripper_grip --open-mm 40
```

Then, with the arm resting on its support and the hand on the power
supply switch:

```bash
uv run python -m muse_qc.gripper.gripper_grip --open-mm 40 --execute
```

After the human types `yes`, the gripper ramps open to the open width
at the `--speed-deg-s` speed (default 15 deg/s), holds 3 s ("place the
part between the jaws now"; contacts in these phases are only printed,
the human's hand may touch the fingers), then ramps toward closed at
the same speed while the newest touch
reading is polled every cycle with a ~2 ms budget. The first reading
with contact on the left AND the right finger (confirmed once with
`safety.check_gripper("closed", ...)`) stops the close, adds the
squeeze, holds 3 s ("grip held at deg ... width about ... mm contacts
..."), then ramps back to the open width, holds 1 s and disables. No
bilateral contact prints "missed grip" and reopens. A reading jump of
more than 20 deg between two cycles, or a reading leaving the safe
range, stops closing, holds the last commanded position for 0.5 s and
disables. Ctrl+C or any error stops closing, opens back to the open
width if the loop still runs, then disables. The arm hold pose (rest
reading clamped into its URDF limits, one `safety.check_target`,
commanded every cycle, 10 % vlims, J6 ref handling) is the same as in
`gripper_motor`.

## 27. x/y correction of the tip

The gripper tip landed about 2 cm off the part in x/y. Three human
measurements of 2026-10-28 show a fixed pattern in the arm base frame
(base x forward, base y left, origin on the joint 1 axis): the real
tip is rotated a little to the LEFT (counter-clockwise seen from
above) and reaches FURTHER OUT than commanded, and the reach error
grows with the distance.

Method: the human placed the white 25 mm cube, the table camera
measured it (`python -m muse_qc.vision.vision_sam "white cube"` gives
table_mm A), the arm hovered at 100 mm over A
(`python -m muse_qc.bringup.hover_move --table-mm A --hold-s 30 --execute`),
the human slid the cube exactly under the center between the finger
tips while the arm held, and after the return the camera measured the
cube again (table_mm B). B is where the real tip was when commanded
to A. Table to base uses `target_pose.table_to_base_mm`
(base_x = table_y - 22.7, base_y = -(table_x - 495.9)).

Data (table mm):

| point | commanded A | measured B |
| ----- | ----------- | ---------- |
| P1 | (394.6, 318.6) | (376.5, 331.3) |
| P2 | (505.0, 348.7) | (491.6, 371.8) |
| P3 | (690.4, 307.6) | (698.2, 333.4) |

`muse_qc/arm/xy_correction.py` (140 lines) models this in base polar
coordinates: real_angle = cmd_angle + YAW_DEG, real_r = cmd_r plus a
radial error R_ERR linear in cmd_r. YAW_DEG is the mean of the three
yaw errors; R_ERR is the least-squares line over the three radial
errors. Outside the fitted radius range R_ERR holds the end value
and never extrapolates. `correct_base_target_mm` inverts the model,
so commanding its output puts the real tip on the part. Offline fit
report (`python -m muse_qc.arm.xy_correction`):

```text
yaw: YAW_DEG=1.94 deg (mean of +2.25, +2.30, +1.25)
radial: R_ERR(cmd_r) = 0.2320 * cmd_r -53.78 mm, fitted cmd_r in [312.8, 345.0] mm
P1: cmd_r=312.8 mm r_err=+18.8 mm residual=1.94 mm
P2: cmd_r=326.1 mm r_err=+21.9 mm residual=2.50 mm
P3: cmd_r=345.0 mm r_err=+26.3 mm residual=4.45 mm
```

The correction is verified only between the smallest and largest
measured radius (312.8 to 345.0 mm from the joint 1 axis). Every
`hover_move` run applies it after the table-to-base conversion and
prints a `corrected base target:` line; `--no-xy-correction` flies
the uncorrected target for A/B tests. Covered by
`tests/test_xy_correction.py` (offline, fake SDK only).

## 28. First pick with the arm (by hand only)

One part (connector, 17 mm wide, 14.5 mm high) lies flat on the table
at a table position the human got from `python -m muse_qc.vision.vision_sam`.
The arm hovers 100 mm over it, opens the gripper, lowers the tip
slowly while watching the finger sensors, closes until the left and
right fingers report contact, lifts the part to the hover height,
holds, puts it back down, opens, lifts and returns to rest. Only the
human runs it, never CI and never an agent.

Three modules share the work (`tests/test_pick.py`, fake SDK, fake
sensor and fake IK only): `muse_qc/pick/pick_plan.py` builds the labeled
arm waypoint plan (trapezoid hover ramp at the CLI speed with
the CLI accel plus a 0.5 s hover settle, 1 mm ladder fast at
40 mm/s above the slow zone and 20 mm/s near the part, lift slow
for the first 10 mm then fast, put-down 2 mm higher, fast raise,
trapezoid return ramp plus 1 s settle, all at 100 Hz),
`muse_qc/pick/pick_loop.py` flies it in one control loop, and
`muse_qc/pick/pick_test.py` is the CLI. The trapezoid ramps start and
stop smoothly instead of jumping to full speed, and the 0.5 s
hover settle lets the joints rest after the ramp before the tip
dives; both soften the visible tip swing, which happens behind
the motor encoders and no gain can see. The gripper opens to
`--open-mm` during the hover ramp (holding at hover until fully
open) instead of in a separate phase afterwards, at 40 deg/s;
closing runs at 40 deg/s until the jaw width reaches 22.0 mm,
then at the old 15 deg/s until contact, so contact detection and
the squeeze behave exactly as before (`muse_qc/gripper/grip_speed.py`;
the 22.0 mm switch sits just above the 17 x 20.5 mm part). The
gripper POS_VEL vlim runs at 25 % of the config value
(0.75 rad/s, about 43 deg/s) so the fast rate fits; the arm vlim
stays at 10 %. The faster gripper and the 20 mm/s approach ran on hardware on 2026-10-30 (see DEVLOG). Every arm
waypoint passes `safety.check_target` per cycle with the grasp floor
`GRASP_Z_MIN_M` = 4 mm on the lower/lift/put-down/raise waypoints
only; everywhere else the 30 mm floor is unchanged, and no safety
check was given up for the speedup. Any contact while lowering
stops the descent at once, holds 0.5 s, climbs back up the same
ladder and returns. `--no-grip` only lowers to `--grip-z-mm` and
holds, so the human can check the real tip height with a ruler
first. `--grip-z-mm` takes 5 to 60 mm (default 8), `--open-mm`
25 to 50 mm (default 28), `--hold-s` 0.5 to 30 s (default 1),
`--speed-deg-s` 2 to 80 deg/s for the free ramps (default 50 in
mit-ff, 15 in posvel); `--accel-deg-s2` 15 to 160 deg/s2 for the
ramp accel (default 100 in mit-ff, 30 in posvel, also used for
the to-box and return ramps); anything else is an argparse error
(exit 2) before anything connects. An explicit value above
15 deg/s needs `--arm-mode mit-ff` (the default): with posvel it
is refused with exit 2 before connecting, because the POS_VEL
vlim at 10 % would cap the motors; without an explicit value
posvel quietly uses 15/30. The agent calls `pick_test` without
these options, so it gets 50/100. The dry run prints the real
values in the `speeds:` line. `--no-xy-correction` flies the
uncorrected target for A/B tests, to measure the uncorrected
error near the part. No gripper-project code is used, only the
serial line protocol and the SDK motor protocol. `--gains`
defaults to `vel2` (the best profile so far, see section 30);
`hold_test` keeps `default` so A/B runs stay explicit.

50/100 is the default since 2026-10-30: the human ran `--no-grip`
stops at 30/60 and 50/100 plus one `--inspect` sort at 50/100,
all without a tracking trip (see DEVLOG). 80/160 is accepted but
not yet run on hardware; the human wants to try it next. The
tracking guard stays the safety net: 80 deg/s at 100 Hz is 0.8
deg per command. 80/160 becomes the default only after a human
test.

Order, with the hand on the power supply switch:

```bash
uv run python -m muse_qc.pick.pick_test --table-mm X Y
uv run python -m muse_qc.pick.pick_test --table-mm X Y --no-grip --grip-z-mm 40 --execute
uv run python -m muse_qc.pick.pick_test --table-mm X Y --no-grip --grip-z-mm 20 --execute
uv run python -m muse_qc.pick.pick_test --table-mm X Y --execute
```

The height correction is verified only at 100 mm so far, hence the
40 and 20 mm ruler stops before the grip run with one part. Parts
must lie 31 to 34.5 cm from the joint 1 axis (the verified x/y
correction range).

The first pick used a grip height of 5 mm (the default then):
at 8 mm only the soft finger tips had touched the 14.5 mm high
part and it slipped out while lifting, at 5 mm it held. The
default is 8 mm since 2026-10-30 (3 mm more margin to the table,
see DEVLOG). The first pick ran with the part placed
by hand under the finger tips (the table camera did not find it
with SAM that day, see DEVLOG):

```text
segments at 100 Hz: hover 2016 lower 950 lift 950 put-down 930 raise 930 return 2116 (total 7892)
hover tip z: 99.8 mm, grip tip z: 5.0 mm
corrected base target: x=+288.35 y=-9.75 mm
xy correction: on
all waypoints ok
contact: gripper commanded deg -45.83 measured deg -49.39 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
stopping: the motors will be disabled now
end readings: arm [-2.79 +0.12 -2.04 +2.94 +2.55 -0.36] gripper -126.28 deg
```

The default arm mode is mit-ff since 2026-10-30 (use
`--arm-mode posvel` for the old POS_VEL loop): the same plan flies
through MIT with per-cycle gravity feed-forward (gains and torque
scale shared with section 30, `--tau-scale` works the same way and
is refused with posvel); the gripper group stays on pos_vel. A tracking
guard trips when any arm joint stays more than 5 deg from its command
for 10 cycles in a row: the loop prints `tracking error on joint N,
holding`, holds the measured pose (a disable would drop the arm in
MIT mode) and walks back to rest with exit 1, inserting a free-speed
ramp first when the safety check would refuse the first abort step.
The dry run prints the MIT gains plus one gravity block each for the
hover pose, the grip pose and the drop pose:

```bash
uv run python -m muse_qc.pick.pick_test --table-mm 500 335 --inspect --arm-mode mit-ff
```

Human result 2026-10-30: with the position trim a mit-ff hold at
tip z 4.7 mm measured 1 mm tip to table by hand (before the trim:
8 mm in mit-ff, 2 mm in posvel); joint 4 read +2.83 deg after the
run (rest +1.89 before, +5.17 before the trim). Decision: mit-ff
is the default since 2026-10-30; use --arm-mode posvel for the old
mode.
The arm sender adds a slow per-joint position trim (about 1/s of the
remaining error, clamped at 4 deg per joint, frozen on a tracking
trip) plus the target step times the loop rate as velocity
feed-forward: the trim only integrates while every joint target moves
at most 5 deg/s, so the kd*v/kp lag of fast moves never winds it up.
`mit trim deg: ...` in the execute output shows the final trim values.

## 29. Find parts on the table (empty reference + SAM confirm)

SAM 3.1 on the full 1920x1080 frame finds the white 25 mm cube
reliably, but not the grey/orange connector parts (about 25 px wide,
low contrast on light wood): on 2026-10-29 the phrases "small part",
"connector", "terminal block", "electrical connector" and "object"
all returned 0 boxes on the zoomed 640 px crop, while "grey object"
confirmed the part. `muse_qc/vision/find_parts.py` therefore works in two
steps: it compares the frame with a reference photo of the empty
table (arm in its rest pose) and takes every new blob as a candidate,
then sends a zoomed crop around each candidate to SAM 3.1 to confirm
it and refine the center. Only the human runs it, never CI and never
an agent (it opens the real table camera and contacts the Meta Model
API over the network).

Order, from the project root:

```bash
uv run python -m muse_qc.vision.find_parts --save-empty
uv run python -m muse_qc.vision.find_parts --no-sam
uv run python -m muse_qc.vision.find_parts
```

First store the reference with the table empty and the arm in its
rest pose: `--save-empty` takes one frame with
`camera_top.open_table_camera` and saves it as
`recordings/empty_table.png`, then prints the path and exits 0.
Then place the parts and run with `--no-sam` first (change
detection only, one line per candidate with blob center, area and
table mm, ending in `candidates: M`), then with SAM (default phrase
`grey object`, overridable with `--phrase`; one SAM call per
candidate on a square crop of side max(3x blob side, 160 px),
upscaled to 640 px). Each line reports the center used (SAM box
center when confirmed, blob center otherwise), the area, the table
position and the SAM verdict (`confirmed` with confidence, `no box`
or `skipped`), sorted by table x; the last lines give the
`parts: N confirmed of M candidates` count and the path of the
annotated copy at `recordings/parts_<YYYYmmdd_HHMMSS>.png` (blob
rectangles, green when confirmed, yellow otherwise, with id and
table mm). The saved parts image is cropped to the table (marker
bounding box plus 15 px); the measurement still uses the full
frame. A missing reference or a size mismatch prints a short
message with exit 2; more than 12 blobs outside the skip zones
prints `too many changes` with exit 1. Skipped blobs never count
against the limit, so gripper and arm motion in the arm zone or a
small light change there cannot trip it; a real light change in
the feed area still does. When the limit trips, the annotated
image is still saved and one extra line prints before the error
(`blobs: N total, S skipped, U unskipped` plus the image path),
so the changes can be seen.

SAM with `grey object` is not deterministic on the zoomed crops, so a
`no box` candidate whose blob area is between 350 and 900 mm2
(`RETRY_AREA_MM2`, real parts measured 398 to 775 mm2) gets one more
confirm with the phrase `small part` (`RETRY_PHRASE`); only a
confirmation from that second call counts, printed as
`sam=confirmed 0.NN (small part)`. Candidates outside the area range
get no second call.

Before SAM, every candidate is checked against three skip zones and
skipped ones cost no SAM call: within 70 mm (max of |dx|, |dy|) of
a sorting box center from `drop_plan.BOXES_TABLE_MM` (parts already
sorted), inside one of the two arm zone rectangles (x 380 to
610 mm, y 0 to 200 mm for the base and arm with cables, x 430 to
590 mm, y 200 to 300 mm for the resting gripper), and farther
than 345 mm from the
joint 1 axis (out of reach, cannot be picked). A skipped candidate
prints `skipped: <reason>` instead of a SAM verdict, the summary
becomes `parts: N confirmed of M candidates (K skipped)`, and the
annotated image draws skipped boxes in grey with the reason.

Real human run with one connector part and the white cube on the
table (2026-10-29): the diff found the part (area about 430 mm2)
while the cube was no candidate (probably merged with its long
shadow above 3000 mm2), and only "grey object" confirmed it:

```text
part 0 center=(1148, 344) area_mm2=431 table_mm=(594.8, 369.4) sam=skipped
candidates: 1
part 0 center=(1146, 340) area_mm2=416 table_mm=(593.8, 371.7) sam=confirmed 0.82
parts: 1 confirmed of 1 candidates
```

Blobs are found by Gaussian blur, absolute difference in LAB space
(max over the channels, threshold 30), morphological open then
close, and external contours; a blob counts only when its area
(through the marker scale) is between 150 and 3000 mm2 and its
center maps inside the table rectangle (0 to 990 mm in x, 0 to
540 mm in y). The table position uses the same calibration and
marker homography as `muse_qc/vision/vision_sam.py`, and the key handling
and error messages match it (the key is never printed).

Take a new reference with `--save-empty` whenever the light or the
camera changes (position, settings, white balance): the diff sees
any illumination shift as new blobs. The reference image stays
local: `recordings/` is git-ignored and never committed.

## 30. Hold test and joint 4/5 gains

While the arm holds a pose (hover, grip, hold), joints 4 and 5
visibly swing back and forth a little (human observation 2026-10-28
and 2026-10-29, also during the x/y measurements). The arm still
reaches its commanded joints within about 0.2 deg on average, so
this looks like a limit cycle of the motor position loop (integral
term plus friction/backlash on the small 4310 motors), not a
planning error. `muse_qc/bringup/hold_test.py` measures it: it flies the
`hover_move` plan (100 mm over the corrected target, same safety
checks per cycle, same 5 deg/s ramp, return to rest and settle,
disable in finally), reads all six joints every cycle while
holding, prints per-joint mean error, peak-to-peak and RMS plus
the dominant swing frequency of joints 4 and 5 (zero crossings of
the error around its mean, `n/a` below 4 crossings), and saves the
samples as `recordings/hold_<YYYYmmdd_HHMMSS>.csv`. Only the human
runs it, never CI and never an agent. From the project root:

```bash
uv run python -m muse_qc.bringup.hold_test --table-mm X Y --hold-s 20
uv run python -m muse_qc.bringup.hold_test --table-mm X Y --hold-s 20 --gains soft --execute
```

`--hold-s` takes 5 to 60 s (default 20); anything else is an
argparse error (exit 2) before anything connects. The dry run
(default) connects, reads, checks every waypoint and moves
nothing. With `--execute` the human confirms with `yes`, then the
arm holds the hover pose for the given time.

`--gains soft` was the first A/B candidate: `muse_qc/arm/arm_gains.py`
keeps the position integral off (pos_ki 0.0) and lowers pos_kp by
30 % for joints 4 and 5 only (70 to 49, 60 to 42; velocity loop
unchanged). The 2026-10-29 A/B (same point, first `--gains
default`, then `--gains soft`, 20 s hold each) showed no visible
difference and soft slightly worse (joint 4 p2p 1.07 deg default
vs 1.25 deg soft, joint 5 p2p 0.15 deg vs 0.35 deg), so default
stays and soft is kept only for reference.

`--gains stiff` is the new A/B candidate after the hand push test
of 2026-10-29 (see docs/DEVLOG.md): pushing the finger tips about
5 mm sideways by hand during the hold moved the wrist along with
the push while the encoders saw it, so the swing is mostly servo
compliance of joints 4/5, not play behind the encoder. Stiff
raises pos_kp by 30 % (x1.3) and vel_kp by 50 % (x1.5) for joints
4 and 5 only; pos_ki and vel_ki are unchanged. The profile is
printed joint by joint before the `yes` prompt, for example
`joint 4: pos_kp 70 -> 91, vel_kp 0.0008 -> 0.0012`. A/B order:
same point, first `--gains default`, then `--gains stiff`, and
during each hold push the finger tips gently about 5 mm sideways
by hand the same way both times; compare peak-to-peak and RMS of
joints 4 and 5 with and without the push.

Safety rule: if the arm buzzes, hums or oscillates on its own
with stiff, switch the power supply off at once and stay on
default.

`--gains vel2` tests the velocity loop only (vel_kp x2.0 for
joints 4 and 5, position loop unchanged). The 2026-10-29 hold
without push showed vel2 clearly calmer than default (joint 4
p2p 0.55 vs 1.07 deg, joint 5 p2p 0.04 vs 0.15 deg, no swing
frequency any more) with no buzzing; with the gentle hand push
the numbers mostly show how far the hand pushed (not
repeatable) and the joints came back exactly (see
docs/DEVLOG.md). Vel2 is therefore the best profile so far and
is now the default of `muse_qc/pick/pick_test.py`; `hold_test` keeps
`default` so A/B runs stay explicit.

`--gains vel3` is the next A/B candidate (vel_kp x3.0 for joints
4 and 5 only, nothing else changed). Test it WITHOUT pushing
first, then compare joint 4/5 peak-to-peak and RMS with the
default run without push; the same safety rule as for stiff
applies (buzz, hum or oscillation on its own means power supply
off at once). Vel3 was tested on 2026-10-29 and not adopted:
without push the motors hummed, and with the push joint 4 rang
at 1.7 Hz instead of coming back cleanly as with vel2 (see
docs/DEVLOG.md); vel2 stays the pick default.

The soft gains live only in a runtime copy:
`write_runtime_yaml` copies the SDK hardware yaml into a
temporary directory (never into the repo, never into the SDK
package) and `make_arm` passes it as `RebotArm(hw_yaml=...)`;
`--gains` exists on `muse_qc/pick/pick_test.py` too (default
`vel2`, same describe lines). The SDK file stays untouched,
and a normal run restores the default gains by itself: the SDK
writes vel_kp/vel_ki/pos_kp/pos_ki into the Damiao registers 25
to 28 on every `mode_pos_vel` call from the loaded yaml values.

`--hold-mode mit-ff` is the second A/B candidate after vel2: the
whole run (start, hover, hold, return, settle) uses the SDK MIT
interface with gravity feed-forward instead of the POS_VEL loop
(`muse_qc/arm/mit_hold.py`). Every arm command is `send_mit(q, vel=0,
kp, kd, tau)` with the DM config gains (joints 1-3 kp 120 kd 8,
joints 4-6 kp 18 kd 2, from `config/rebotarm_dm.yaml` in the SDK
checkout) and `tau = 1.0 * g(q)` from the SDK
`compute_generalized_gravity` on the cached robot model. The
mode switch happens once before enable, never mid-run; the
first command is the measured start pose; the gripper group is
never touched. A tracking guard aborts like Ctrl+C (hold, then
settle and disable in `finally`) when any arm joint stays more
than 5 deg from its commanded position for 10 cycles in a row,
printing `tracking error on joint N, holding`. Header and CSV
name carry the mode (`hold mode: mit-ff`, `hold_mit-ff_*.csv`),
all other printouts are unchanged so both modes compare
directly. The dry run (default, no `--execute`) prints the MIT
gains, the torque scale and the gravity torque at the hover pose
per joint in N*m, so the human checks values and signs before
moving. Check first without moving:

```bash
uv run python -m muse_qc.bringup.hold_test --table-mm 506.4 338.1 --hold-s 20 --gains vel2 --hold-mode mit-ff
```

The comparison run is one human hardware run per mode, same
point, same gains, 20 s hold each:

```bash
uv run python -m muse_qc.bringup.hold_test --table-mm 506.4 338.1 --hold-s 20 --gains vel2 --hold-mode mit-ff --execute
uv run python -m muse_qc.bringup.hold_test --table-mm 506.4 338.1 --hold-s 20 --gains vel2 --hold-mode posvel --execute
```

Recorded runs (2026-10-30, same point 506.4 338.1, `--gains vel2`, 20 s hold each; recorded output, never edited):

```text
mit-ff:
hold commanded deg: -3.84 -102.77 -95.99 +83.23 -0.00 +0.01
hold measured deg: -3.44 -102.85 -94.89 +82.67 +0.34 -0.32
hold measured tip mm: x=+293.6 y=-19.0 z=+95.1
motion finished, arm holds the settle pose
joint 1: mean err +0.40 deg, p2p 0.00 deg, rms 0.40 deg
joint 2: mean err -0.08 deg, p2p 0.26 deg, rms 0.08 deg
joint 3: mean err +1.10 deg, p2p 0.17 deg, rms 1.10 deg
joint 4: mean err -0.56 deg, p2p 0.39 deg, rms 0.56 deg, swing n/a
joint 5: mean err +0.34 deg, p2p 0.00 deg, rms 0.34 deg, swing n/a
joint 6: mean err -0.32 deg, p2p 0.00 deg, rms 0.32 deg
```

```text
posvel:
hold commanded deg: -3.84 -102.77 -95.99 +83.23 -0.00 +0.01
hold measured deg: -3.86 -102.78 -96.01 +82.89 -0.01 -0.01
hold measured tip mm: x=+297.9 y=-20.1 z=+99.6
motion finished, arm holds the settle pose
joint 1: mean err -0.01 deg, p2p 0.04 deg, rms 0.02 deg
joint 2: mean err +0.00 deg, p2p 1.31 deg, rms 0.11 deg
joint 3: mean err +0.00 deg, p2p 1.20 deg, rms 0.10 deg
joint 4: mean err -0.01 deg, p2p 1.11 deg, rms 0.20 deg, swing 0.8 Hz
joint 5: mean err -0.01 deg, p2p 0.00 deg, rms 0.01 deg, swing n/a
joint 6: mean err -0.02 deg, p2p 0.15 deg, rms 0.03 deg
```

Result in one paragraph: with MIT plus gravity feed-forward joint 4 did not swing (p2p 0.39 deg, standard deviation 0.023 deg from the CSV, no swing frequency found) while the position-velocity hold swung at 0.8 Hz with p2p 1.11 deg; joints 2 and 3 were also much calmer (p2p 0.26/0.17 vs 1.31/1.20 deg). The human watched both runs: MIT hold looked steady, posvel swung again. Cost: MIT has a steady offset because the gripper is heavier than the SDK model (no integral term): joint 3 +1.10 deg, the tip about 5 mm lower (z 95.1 vs 99.6 mm). Next step: per-joint torque scale to remove the offset before MIT is used for picking. CSV files: recordings/hold_mit-ff_20261030_001308.csv and recordings/hold_posvel_20261030_001520.csv (not in the repo).

`--tau-scale S1 S2 S3 S4 S5 S6` sets the per-joint gravity torque
scale for `--hold-mode mit-ff` (six values, each 0.0 to 2.0,
default 1.0 1.05 1.45 1.0 1.0 1.0, so `tau = scale * g(q)` per joint). Anything else
is refused before anything is enabled, and `--tau-scale` with
`--hold-mode posvel` is an argparse error (exit 2). The dry run
prints the scale (`tau scale: 1.00 ...`) and the scaled torque per
joint (`joint 3 gravity torque: -5.564 N*m, scaled -7.790 N*m`),
and every run prints `hold tip z offset: ... mm` (measured minus
commanded tip z) next to the tip line. First candidate against the
joint 3 offset above:

```bash
uv run python -m muse_qc.bringup.hold_test --table-mm 506.4 338.1 --hold-s 20 --gains vel2 --hold-mode mit-ff --tau-scale 1.0 1.0 1.4 1.0 1.0 1.0
```

Recorded runs (2026-10-30, same point 506.4 338.1, `--gains vel2`, 20 s hold each; recorded output, never edited; see docs/DEVLOG.md for the full blocks): with `--tau-scale 1.0 1.0 1.4 1.0 1.0 1.0` the tip z offset was -2.0 mm (joint 3 mean err +0.23 deg); with `--tau-scale 1.0 1.05 1.45 0.25 1.0 1.0` the hold was close but joint 4 sagged on the way back and the tracking guard stopped the run after 5203 of 6229 cycles, so a joint 4 factor below 1.0 only fits the hover pose; with `--tau-scale 1.0 1.05 1.45 1.0 1.0 1.0` the tip z offset was +0.0 mm with no joint 4 swing and no guard stop. The last set is now the default `TAU_SCALE` in `muse_qc/arm/mit_hold.py` (joint 4 stays at 1.0 because its model error depends on the pose), so the dry run prints the scaled lines by default.

## 31. Sorting into the good and bad box

Two open boxes (100 x 100 mm, 70 mm high, open on top) stand on the
table: the left box takes good parts, the right box takes rejects.
Their centers (top rim in the camera view) were measured by the
human with `python -m muse_qc.vision.vision_sam "box"` as table mm
(376.4, 423.8) for good and (675.4, 420.3) for bad.
`muse_qc/pick/drop_plan.py` keeps these positions plus the nominal
170 mm IK tip height and the 30 deg tool tilt. The arm correction
puts the real tip about 40 mm lower, about 130 mm above the table,
i.e. about 60 mm above the rim; below 110 mm real tip height
(40 mm above the rim) the solve refuses with "drop tip too low".

A vertical tool does not reach the boxes at that height (about 360
to 420 mm reach at 110 to 140 mm tip height), so the drop pose tilts
the tool 30 deg away from the joint 1 axis in the radial direction
(the finger tips point outward, away from the arm). `solve_drop`
uses the same SDK IK machinery as the hover solve but applies no x/y
correction: the correction was fitted only for r 312.8 to 345 mm,
and the wide box opening forgives 2 to 3 cm. The carry ramp
(`build_drop_segments`) refuses the plan with "transit too low"
unless every to-box waypoint keeps the corrected tip at or above
90 mm (unchanged); there is no touch-guarded put-down in the box,
the gripper just opens over it. An abort during to-box walks the
remaining carry waypoints to the drop pose and then returns, during
release it returns straight away; the gripper is never opened on an
abort.

With `--drop good|bad` the pick plan is hover, lower, lift, to-box,
release, return (no put-down, no raise); the lift still ends at the
hover height. During to-box the loop keeps the part-lost watch
exactly as during lift, then opens the gripper to 40 mm over the
box, holds 1 s, and reads the touch sensors once: no contacts means
the part dropped into the box, contacts left mean it is still held
(warning only). Without `--drop` the behavior is unchanged
(put-down). Only the human runs it, never CI and never an agent.

Order, with the hand on the power supply switch:

```bash
uv run python -m muse_qc.vision.find_parts --save-empty
uv run python -m muse_qc.pick.pick_test --table-mm X Y --drop good
uv run python -m muse_qc.pick.pick_test --table-mm X Y --drop good --execute
```

Save the empty-table reference again with the boxes in place first:
`find_parts --save-empty` with the table without parts and the arm
at rest, otherwise the boxes themselves become candidates. Parts
must lie between the arm and the boxes within about 345 mm of the
joint 1 axis, the outer range of the x/y correction.

## 32. Inspect in the gripper and sort (by hand only)

With `--inspect` the pick sorts the part into the box the lever
count decides instead of a fixed `--drop` box (`--inspect` and
`--drop` exclude each other on the command line). The plan is
built once with both drop branches: hover, lower and lift as
today, then the to-box/release/return waypoints for the good and
for the bad box. The dry run solves and checks every waypoint of
both branches before anything moves, and first checks that the
arm color node exists and that the API key is present (the key is
never printed).

Order, with the hand on the power supply switch:

```bash
uv run python -m muse_qc.pick.pick_test --table-mm 513.3 340.7 --inspect
uv run python -m muse_qc.pick.pick_test --table-mm 513.3 340.7 --inspect --execute
```

Recorded lines (2026-10-29, good part, from the earlier hover
flow):

```text
inspect: levers 3 verdict good
contact: gripper commanded deg -45.82 measured deg -49.36 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
dropped into the good box
```

The bad part run gave "inspect: levers 2 verdict bad" and "dropped into the bad box".

With `--execute` the loop no longer stops at hover: it walks the
hover segment (opening the gripper) and lowers directly, grips,
lifts and holds the last lift waypoint with the part held: first
2 s to let the arm settle (run on hardware 2026-10-30; raised
from 1 s because the 2026-10-30 ~10:26 photo came out blurred),
then the main thread grabs one frame
with the arm camera, saves it as
recordings/armcam_<timestamp>.png, counts the levers with
count_clips.count_frame and hands the verdict to the loop, which
takes the ranges of that box and walks its to-box waypoints. With
the gripper flow the same lines print in pick order: the grip
contact, the lifting contacts, "part lifted, holding",
"inspect: levers 3 verdict good", "dropped into the good box".
The camera sits on the gripper, so a held part always appears at
the same image position: the inspection only accepts the part
near `EXPECTED_PART_PX` = (342, 290) (up to `MAX_PART_OFFSET_PX`
= 60 px away) and counts its levers with `LEVER_PROMPT`
("brown bar"); otherwise it reports "part: target not in view"
and the unverified part is sorted into the bad box (see below).
The control loop thread never opens the camera
and never calls the API; without `--inspect` everything behaves
exactly as today.

Failure behavior: no frame, no part found or an API error prints
"inspect failed: ..." and the unverified part is sorted into the
bad box with exit code 1 -- the loop prints "inspect failed,
sorting into the bad box (unverified part)" and never opens the
gripper above the table; an unverified part counts as reject. The
same bad-box sort fires when no verdict arrives within 60 s, with
"inspect timeout, sorting into the bad box (unverified part)".
A missed grip, a part lost while lifting, a
contact stop or any other abort before the lift ends skips the
inspection ("inspect skipped: no part held") and returns as
today. Only the human runs it,
never CI and never an agent.

Lever-count retry (run on hardware 2026-10-30): SAM is not fully
deterministic (the same image gave 0 levers three times, then
1). A first count of 1 or 2 levers is final (no retry), only 0
or more than 3 are retried. The
first retry is one fresh arm camera frame with the normal
prompt, then ("brown bar", 6, 15), ("brown bar", 10, 25),
("orange bar", 8, 15) and ("orange bar", 6, 15) on the original
frame. The part is good only when two retry counts each give
exactly 3; the run stops at the second 3, and also stops early
when the remaining retries can no longer reach two 3s.
Otherwise the first result stands (verdict bad), so an
unconfirmed single 3 never turns a bad part into good. A first
count that misses the part ("part: target not in view") is
retried first on up to `PART_MISS_RETRIES` = 2 fresh frames
before those lever retries. A retry
prints "inspect retry: N attempts" before the "inspect: levers
..." line. The worst case is 6 counts (12 SAM calls) plus a
second camera open, so the inspect timeout is 60 s; the arm
keeps holding the part meanwhile.

A saved image can be rechecked by hand with
`uv run python -m muse_qc.inspection.count_retry --image <file>` (not yet
run on hardware): it always uses --expect-target behavior,
never grabs a fresh frame, and prints one line per attempt
("attempt N: prompt P scale S margin M levers L") plus
"levers: N" and "verdict: ...".

## 33. Agent loop with Muse Spark (by hand only)

`muse_qc/agent/agent.py` runs the quality inspection cell on its own
through Meta Muse Spark (model `muse-spark-1.3`, Responses API,
tool calling): the model decides which tool to call next; our code
only executes the tools and enforces safety. The model never sends
coordinates and never moves the arm directly: positions always
come from our own latest scan (the model only passes part IDs),
and every motion goes through the `pick_test --inspect --execute`
path with its safety checks. Each sort runs as its own
`python -m muse_qc.pick.pick_test` child process, so the operating
system frees the serial port when the sort exits. The pick
confirmation is passed through only after the human typed "yes"
once for the whole run.

Tools offered to the model:

- `scan_table()` lists the SAM-confirmed, not skipped parts of the
  latest table scan under fresh IDs.
- `inspect_and_sort(part_id)` picks the part with that id, counts
  its levers and drops it into the good or bad box. Unknown or
  stale IDs are refused; after every sorted part the table is
  scanned again right away (the arm is back at rest with the
  motors off, since each sort runs as its own child process) and
  the new part list comes back inside the tool result under
  "table after sort" with new IDs, so a slipped part gets its new
  position, the live page shows the table after every sort and no
  extra model call is needed. The rescan stays cheap: parts
  already confirmed in the previous scan are not sent to SAM
  again (match within `KNOWN_MATCH_MM` = 10.0 mm, confidence
  copied). A failed or missed sort, a failed rescan and an empty
  rescan expire the whole scan, so the model calls `scan_table`
  then.
- `write_report()` builds the HTML report from only this run's
  records.
- `finish(summary)` ends the run; the summary is printed and stored.

Hard limits enforced by the code, not by the model: at most 40
model calls, at most 10 sorts, at most 2 `inspect_and_sort` calls
per part position (positions within 15 mm count as the same
part), stop after 2 consecutive "error"/"inspect_failed"
outcomes. Ctrl+C during a pick aborts the pick safely and stops
the whole run ("agent: interrupted by human, stopping") instead of
starting the next pick. Every step is printed (`agent: ...` model
text, `agent -> ...` tool calls, one-line tool results, plus
`agent: tokens in N out M` per model call when the API reports
usage) and saved to `recordings/agent_<timestamp>.json` (model
text, tool calls, tool results; no key, no reasoning blobs). An
API error prints a short message without the key, stops the loop
and still writes the transcript. The command exits 0 only when
the agent called `finish`, else 1 (limit, failures, API error,
interrupt); exit 2 means a missing key or a missing "yes" with
nothing moved.

Run the simulation first: the real Muse Spark API with fake tools
(a fake table with 3 parts, two good and one bad, one missed grip
on the first try of one part), so the tool calling can be checked
without hardware:

```bash
uv run python -m muse_qc.agent
```

Then, with the hand on the power supply switch:

```bash
uv run python -m muse_qc.agent --execute
```

`--execute` asks once `Type yes to let the agent move the arm:`
and only `yes` starts; anything else moves nothing. The Responses
API function-calling format in `muse_qc/agent/agent.py` was confirmed by the human runs on 2026-10-29 and 2026-10-30 (see DEVLOG). Every response item is converted
to a plain dict and sent back for the next call, including
reasoning items with their encrypted content (the transcript
holds no reasoning blobs). Only the human runs it, never CI
and never an agent.

## 34. Lever direction on the table camera (run on hardware 2026-10-30)

The human places parts at any angle; later the arm should turn
joint 6 so it grips every part the same way as a straight part.
`muse_qc/vision/part_angle.py` measures the lever direction first,
vision only (moves nothing): the precise angle comes from the
part outline (`minAreaRect` of the diff-mask blob contour), and
a stripe cue inside the part only picks which of the two
outline directions (angle or angle + 90) the bars follow. The
stripe cue is the structure tensor of the grayscale gradients
(Sobel) inside the filled diff contour eroded by
`REGION_ERODE_PX` = 2, weighted only where the pixel is
lever-like or next to one (the lever HSV mask dilated by
`WEIGHT_DILATE_PX` = 2, covering the dark gaps between merged
bars): the dominant gradient runs across the bars, so the lever
direction is the tensor direction + 90 deg, with strength =
coherence ((l1 - l2) / (l1 + l2)). `MIN_STRENGTH` = 0.3 only
decides that outline-vs-+90 choice; the final angle always
comes from the outline. The lever HSV range is tight (S at
least 110, V at most 165) because the wooden table itself falls
into the lever hue (see docs/DEVLOG.md 2026-10-30): no wood
pixel outside the contour may influence the direction.
The result is mapped into the table frame and reported as
`lever_deg` in [0, 180), where 0 means bars parallel to table
+x and 90 parallel to table +y. The cue order is slot, levers,
outline: a small dark slot at one end of the levers is tried
first, then the stripe cue, then the long outline side when its
aspect ratio reaches `FALLBACK_ASPECT_MIN` = 1.1, else the
direction is unknown.

`muse_qc/vision/find_parts.py` prints the direction on each unskipped
part line (`lever_deg=NN.N (levers|slot|outline)`, or
`lever_deg=? (reason)` when unknown, for example
`lever_deg=? (bars unclear)`), with `--no-sam` and `--image`
too, so the human can test on saved images and live without
SAM cost:

```bash
uv run python -m muse_qc.vision.find_parts --no-sam
uv run python -m muse_qc.vision.find_parts --image recordings/frame.png --no-sam
```

First hardware run 2026-10-30 (see docs/DEVLOG.md): the old
crop-based lever cue was decided or blocked by wood pixels and
no real part got source "levers"; fixed by the inside-the-part
stripe cue above. A small dark slot at one end of the levers
(V at most 70) is tried before the stripes and picks outline vs
outline + 90 instead (printed as `(slot)`): on the small real
parts the lever-end and edge gradients can push the stripe cue
past `MIN_STRENGTH` in the wrong direction (see docs/DEVLOG.md
2026-10-30 ~13:30), while the slot was right on all three parts.
Since 2026-10-31 the slot long axis is tried before that
(`muse_qc/vision/slot_axis.py`: dark V at most 60 inside the filled
contour, 8-connected components of 5 to 40 px with elongation
1.6 to 4.5, nearest the part center; long axis + 90 deg is the
lever estimate, the agreeing outline direction within 15 deg
wins else the estimate itself, printed as `(slot axis)`):
its own orientation ignores the shadow that shifted the old
center-to-slot cue by 45 to 90 deg on the 2026-10-31 table,
and the much longer open-lever gap never passes the
elongation limit.

## 35. Grip yaw from the lever direction (run on hardware 2026-10-30)

`muse_qc/pick/grip_yaw.py` turns the `lever_deg` of section 34 into a
grip yaw offset: the lever direction is mapped into the arm base
frame (`table_to_base_mm` on the part center and center plus the
unit lever direction), compared with the radial yaw
`atan2(base_y, base_x)` of the part, and wrapped modulo 180 into
[-90, 90), so a 180 deg turn of the part does not matter. The
offset is clamped to +/- `MAX_GRIP_YAW_DEG` = 70 (joint 6 camera
cable; `safety.check_target` still checks every waypoint against
the 75 deg J6 limit). `muse_qc/pick/pick_test.py` takes it as
`--lever-deg DEG` (0 to 180, optional; without it nothing
changes): the dry run prints
`grip yaw: lever NN.N deg, offset +NN.N deg` (plus ` (clamped)`
when clamped) before the plan summary, and the same offset runs
through `solve_hover`'s `yaw_offset_deg` for hover, lower, lift
and put-down. The agent passes each part's `lever_deg` from the
scan to `real_sort`, which adds `--lever-deg` to the `pick_test`
child command only when it is known. Dry run for the human:

```bash
uv run python -m muse_qc.pick.pick_test --table-mm 643.9 297.2 --lever-deg 31.9 --inspect
```

## 36. Live page (run on hardware 2026-10-30)

`muse_qc/agent/live_page.py` (`LivePage`, 256 lines) shows the agent
run while it happens, for the human watching and for the demo
video. It keeps the run state (status, counters, latest table
image, part list, sorted part cards newest first, last 15
console lines) and rewrites `recordings/live.html` after every
update, atomically (temp file in the same folder plus
`os.replace`), so the browser never shows a half-written page.
The page is static HTML only (no server, no JavaScript, no
external URLs) with `<meta http-equiv="refresh" content="2">`,
large dark-on-light fonts for 1920x1080 next to the arm. All
text is html-escaped and image paths are relative to the page
folder (as in `report.py`); the GOOD/BAD badges reuse
`report._badge`. A failed write prints one short
`live page: <error>` line and the run goes on.
`muse_qc/agent/agent.py` (399 lines) takes `live=None`: when given,
every printed line also goes to `live.line`, the status reads
"scanning the table" / "sorting part N" / "writing the report"
before each tool call, `live.scan` follows a successful scan,
`live.sorted` follows every sort result (with the record crop
image when its JSON is readable), and `live.finish` closes the
run, also after a stop or failure (with the stop reason).
`main` creates `LivePage()` for both `--execute` and the
default simulate mode and prints
`live page: recordings/live.html (open it in the browser)`
once at the start. How to watch:

```bash
uv run python -m muse_qc.agent
firefox recordings/live.html &
```

If no default browser is set, xdg-open fails; open the file with the browser directly.

The page reloads every 2 s on its own. First used on hardware 2026-10-30 (see DEVLOG).

Order by chat (human request 2026-10-30): `main` takes
`--task "TEXT"` (optional) and `--chat`. The task text replaces
`START_TEXT` as the first user message, so the human gives the
order in plain words; with `--chat`, `main` asks
`next order (empty line to quit): ` after each run and a
non-empty line continues the same run with that line as a new
user message (same history, same `RunState`, same limits and
safety; the `yes` confirmation is asked only once at the start).
The chat logic lives in `muse_qc/agent/agent_chat.py` (the transcript
writer moved there from `agent.py` to keep it within 400 lines).
Each human order and the final summary go through the console
output, hence onto the live page. The tools and their refusals
are unchanged: the order can only change WHICH parts the model
asks for, never bypass a tool check:

```bash
uv run python -m muse_qc.agent --execute --chat --task "sort all parts"
```

Without `--task`, `--chat` asks for the order first, so the human types it live:

```bash
uv run python -m muse_qc.agent --execute --chat
```

## 37. Inspection station (confirmed on hardware 2026-10-31)

With `--station` (needs `--inspect`) the arm carries the held
part to one fixed point left of the table after the lift, turns
joint 6 to one fixed angle and holds there while the main
thread inspects, so every part is seen under the same light
against the same background; both drop branches then start
from the station instead of the lift top. The constants live
in `muse_qc/pick/inspect_station.py`: `STATION_TABLE_MM` =
(235.0, 150.0) table mm (human chosen 2026-10-30, about 290 mm
from the joint 1 axis), `STATION_TIP_Z_MM` = the hover height
(100 mm), `STATION_J6_DEG` = 0.0 (absolute joint 6 angle, the
human tunes it later), `USE_STATION_DEFAULT` = True since the human 2026-10-31 00:08 run confirmed the station on the real arm, so `--inspect` and the agent sort at the station by default and `--no-station` turns it off. An unreachable station or
a joint 6 outside its limits refuses the whole plan before
anything moves; an abort during to-station or at the station
returns to rest over planned, already checked waypoints with
the gripper unchanged (the part stays held, never opened
above the table). Dry run first, then one part with the
hand on the power supply switch:

```bash
uv run python -m muse_qc.pick.pick_test --table-mm X Y --inspect --station
uv run python -m muse_qc.pick.pick_test --table-mm X Y --inspect --station --execute
```

The dry run prints
`station: table mm (235.0, 150.0), tip z 100.0 mm, joint 6 +0.0 deg`
plus the to-station length in the segments line. Only the
human runs it, never CI and never an agent.
To tune the joint 6 angle without editing code, pass `--station-j6 DEG` together with `--station` (refused without it); the dry run then prints the angle actually used, and the angle that looks best is written into `STATION_J6_DEG` in `muse_qc/pick/inspect_station.py`:

```bash
uv run python -m muse_qc.pick.pick_test --table-mm X Y --lever-deg L --inspect --station --station-j6 15 --execute
```

## Safety notes

Read-only review of the SDK file
`reBotArm_control_py/actuator/rebotarm.py` (SDK clone at
`~/meta-hackathon/seeed-sdk`) and of `muse_qc/arm/arm.py`. No hardware was touched.

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
  starts no thread and commands no motion. This is the path `muse_qc/arm/arm.py`
  uses for `read_joints()` and `read_gripper()`.
- `disconnect()` (`rebotarm.py:738-750`) stops the control loop
  (`stop_control_loop`, `rebotarm.py:816-820`), then calls
  `disable_all()` (`rebotarm.py:658-664`, per group
  `rebotarm.py:294-303`), waits, and shuts the bus controllers down
  (`shutdown()` / `close()`, `rebotarm.py:744-747`). After
  `disable_all()` the motors no longer hold position, so the arm can sag
  under its own weight.
- The `muse_qc/arm/arm.py` wrapper adds no thread and no bus traffic of its own:
  `Arm.__init__` only stores the factory,
  `Arm.connect` only calls `sdk.connect()`, and
  `Arm.disconnect` only calls `sdk.disconnect()`.
- Not verified: the exact bytes the `motorbridge` package
  (`Controller.disable_all`, `shutdown`, `close`, `send_*`) puts on the
  bus. That package was outside this review.

## Run the offline suite in Docker or CI

`scripts/setup_sdk.sh` automates sections 1 to 3 above, so a fresh clone
only needs the script plus `uv sync`. GitHub runs the same steps on every
push to main (`.github/workflows/ci.yml`): setup script, `uv sync
--locked`, `uv run pytest -q`, an import check of every `muse_qc` module,
and `uv run python -m muse_qc.agent --help`. No secrets, no
`MODEL_API_KEY`, no network calls in tests, no hardware.

The same offline suite runs in Docker, without any local setup:

```bash
docker build -t muse-qc-cell .
docker run --rm muse-qc-cell
```

The agent with a fake table (real Muse Spark, no arm) takes the key at run
time only; it is never baked into the image:

```bash
docker run --rm --env-file .env muse-qc-cell uv run python -m muse_qc.agent
```

## Export a run for the demo page

This step reads existing inspection records and copies crop images only.
It is offline: no camera, no arm, no API key, no network. From the project
root:

```bash
uv run python -m muse_qc.inspection.export_site --recordings recordings --since <YYYYmmdd_HHMMSS> --until <YYYYmmdd_HHMMSS> --out site/data
```

The command writes `site/data/run.json` (date plus one entry per part in
order: time, levers, verdict, box and retries only when recorded, and the
relative crop image path) and re-encodes only the enlarged crop images
(`count_*_crop.png`) as JPEG into `site/data/media/`. It also collects the
table rescans (`recordings/parts_<stamp>.png`) in the same window and links
each part to the newest rescan taken before it was picked, plus a final
rescan after the last part. It prints the part,
good and bad counts, the table scan count, plus the total size in KB.
