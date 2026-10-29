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
centre of marker 0, +x towards the centre of marker 1, units mm. The
four points are a least-squares fit (residuals under 0.5 mm) to all
six hand-measured centre-to-centre distances of the black marker
squares: ID0-ID1 982 mm, ID2-ID3 977 mm, ID0-ID2 531 mm,
ID1-ID3 530 mm, diagonals ID0-ID3 1117 mm and ID1-ID2 1110 mm. The
markers form a slightly skewed quadrilateral, not a rectangle. From
the project root:

```bash
uv run python -m muse_qc.table_coords
```

Expected result: one line per ID with centre in pixels, centre in
millimetres and side length in millimetres (expected 40 mm), then the
table position of the image centre, then the path of one grid copy
(grid lines every 100 mm over the marker quadrilateral, origin and
axes labelled) at `recordings/table_grid_<YYYYmmdd_HHMMSS>.png`.
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

## 15. Model API connection check (by hand only)

This step contacts the Meta Model API over the network, so only the
human runs it, never CI and never an agent. `muse_qc/model_check.py`
reads `MODEL_API_KEY` from `.env` or the environment (the key is never
printed, not even partly), calls `GET /v1/models` once through the
`openai` SDK with the base URL `https://api.meta.ai/v1`, prints the
number of models and one `OK`/`MISSING` line each for `muse-spark-1.3`
and `sam-3.1`. The exit code is 0 only when both are listed. From the
project root:

```bash
uv run python -m muse_qc.model_check
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
agent. `muse_qc/vision_sam.py` takes one frame with
`open_table_camera()` from `muse_qc/camera_top.py` (or a saved image
with `--image`), sends it with a noun phrase to SAM 3.1 through the
`openai` SDK in one `POST /v1/responses` call, parses the box records
from the output text (masks are ignored) and draws the boxes with ids
onto a copy of the image. The key is read with
`muse_qc/model_check.py` and never printed. From the project root:

```bash
uv run python -m muse_qc.vision_sam "small part"
uv run python -m muse_qc.vision_sam "small part" --image path.png
```

Result format: one line per box with id, box corners, center in
pixels and confidence, then the count, then the path of the annotated
copy at `recordings/sam_<YYYYmmdd_HHMMSS>.png`. Before the box lines,
one `table mapping:` line reports `calibrated`, `raw pixels` (no
calibration file) or `unavailable (<reason>)`, and each box line ends
with `table_mm=(x, y)` in millimetres (the box centre mapped onto the
table frame on the same frame sent to SAM) when the mapping works.

Real human run of `uv run python -m muse_qc.vision_sam "small part"`
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
runs it, never CI and never an agent. `muse_qc/fk_check.py` reads
the six joint angles once through `muse_qc/arm.py` (read only: it
never enables, zeroes or sends anything), computes the `end_link`
pose with the Seeed SDK kinematics (metres and radians in the SDK,
printed as mm and degrees) and projects the finger-tip position
30 mm along the tool axis (link6 to end_link). The 30 mm value
(`TIP_OFFSET_MM`) is an estimate from a hand measurement (185 mm
from the joint 6 mounting face to the finger tips, minus the
155.39 mm URDF `end_joint` offset) and must be verified on the arm.
From the project root:

```bash
uv run python -m muse_qc.fk_check
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

Program: `muse_qc/first_move.py` with the plan in
`muse_qc/first_move_plan.py`, checked by
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
uv run python -m muse_qc.first_move
uv run python -m muse_qc.first_move --execute
```

## 19. Arm camera test (by hand only)

This step opens the real arm camera, so only the human runs it, never CI
and never an agent. `muse_qc/camera_arm.py` finds the colour stream of
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
uv run python -m muse_qc.camera_arm
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
never CI and never an agent. `muse_qc/count_clips.py` works in two
steps on a saved arm camera image (for example from section 19).
First it sends the full image with `PART_PROMPT` ("small part") to
SAM 3.1 through the `muse_qc/vision_sam.py` request code, ignores
boxes in the gripper zone (box center y at or above
`GRIPPER_ZONE_Y` = 380 px; the gripper is fixed in the arm camera
image) and picks the remaining box with the highest confidence at
or above `MIN_CONFIDENCE` (0.5). Then it crops that box plus
`CROP_MARGIN_PX` = 15 on each side (clamped to the image), enlarges
the crop `CROP_SCALE` = 8 times with `cv2.INTER_CUBIC`, and sends
the crop with `LEVER_PROMPT` ("orange bar") to SAM 3.1 again. Lever
boxes at or above confidence 0.5 are kept, overlapping duplicates
are merged (IoU above 0.5 keeps the higher confidence), and boxes
shorter than `MIN_BAR_HEIGHT_FRAC` = 0.3 of the crop height are
dropped as torn-out remnants (printed as
`dropped (too short): ...`). The program
prints `part: box=(...) confidence=...`, one line per kept lever
box with its confidence, then `levers: N` and
`verdict: good|bad` (good means exactly `EXPECTED_LEVERS` = 3
levers), and saves two annotated copies under one timestamp:
`recordings/count_<YYYYmmdd_HHMMSS>.png` (full image with the part
box and `levers: N`) and
`recordings/count_<YYYYmmdd_HHMMSS>_crop.png` (enlarged crop with
the lever boxes), checking both write results. With `--image`,
`muse_qc/vision_sam.py` skips the table mapping and prints
`table mapping: skipped (image file)`. The API key comes only from
the existing `.env` loading code and is never printed. From the
project root:

```bash
uv run python -m muse_qc.count_clips --image recordings/armcam_test.png
uv run python -m muse_qc.count_clips --image recordings/armcam_test.png --prompt "orange bar"
```

`--prompt` overrides only the lever prompt; the part prompt is
always `PART_PROMPT`. All values above are constants at the top of
`muse_qc/count_clips.py`.

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

This step turns a part position in table millimetres (as printed by
`muse_qc/vision_sam.py`, table frame of `muse_qc/table_coords.py`:
origin at the centre of marker 0, +x towards marker 1, +y towards
marker 2 away from the arm) into an arm pose with the gripper tip
straight above the part at a hover height. `muse_qc/target_pose.py`
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
uv run python -m muse_qc.target_pose --table-mm 495.9 322.7 --start home
```

Then with the arm power supply on (still no motion, reads the
current joints once through the read-only `muse_qc/arm.py`):

```bash
uv run python -m muse_qc.target_pose --table-mm 495.9 322.7 --start arm
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

Program: `muse_qc/hover_move.py`, checked by
`tests/test_hover_move.py` (fake arm only). It moves the gripper
tip from the current rest pose to the hover pose above a table
target (the same pose `muse_qc/target_pose.py` already solves and
checks: table frame of `muse_qc/table_coords.py`, IK seeded with
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
uv run python -m muse_qc.hover_move --table-mm 495.9 322.7
uv run python -m muse_qc.hover_move --table-mm 495.9 322.7 --execute
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

Arm tip correction (`muse_qc/arm_calibration.py`, checked by `tests/test_arm_calibration.py`): from the human hold measurements of 2026-10-28 (docs/DEVLOG.md) the code corrects three small errors, so a planned target lands where the real tip goes. The base sits 5 mm lower (`BASE_Z_MM = -5.0`), the gripper reaches 190 mm from the end of motor 6 (tip offset beyond end_link derived from the model, 155.39 mm), and joint 3 gets `true_q3 = reported_q3 + k * q3` with `k = -0.0556` (ratio form, fit RMS 5.45 mm; load form RMS 6.76 mm). `first_move_plan.real_tip_m` returns this corrected tip, so every `safety.check_target` sees the real height above the table. `target_pose.solve_hover` iterates the existing IK (`solve_corrected`: solve nominal, shift by the corrected error, stop below 0.5 mm or after 10 iterations), and `target_pose.run` plus `hover_move` print one extra `corrected tip table mm: ... (iterations n)` line. The 80 mm hover floor in `hover_move` stays. On 2026-10-28 the human verified the correction at (495.9, 322.7) and (495.9, 242.7) at 100 mm: both measured 100 mm (docs/DEVLOG.md). Heights below 100 mm are not verified yet. The nominal 150 mm hover at (495.9, 322.7) is unreachable once corrected (joint 4 limit).

## 24. Gripper touch sensors (by hand only)

The gripper is pre-existing hardware: Fin-Ray fingers on the reBot
gripper jaws with four magnet-plus-magnetometer sensors (L1, L2 left,
R1, R2 right), read by a Raspberry Pi Pico over USB serial. Only the
reader (`muse_qc/gripper.py`, checked by `tests/test_gripper.py`) is
new in this hackathon; hardware and firmware are not part of it. No
firmware code is used, only the documented serial line protocol, so
this MIT repository stays clear of the gripper project's GPL-3.0 code.

Run (human only, moves nothing; the tool never commands the arm;
the reader is `muse_qc/gripper.py`, the watch CLI moved to
`muse_qc/gripper_watch.py` so the reader stays at or below 300 lines):

```bash
uv run python -m muse_qc.gripper --watch 10
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
and never an agent. `muse_qc/gripper_motor.py` (checked by
`tests/test_gripper_motor.py`, fake SDK only) needs `--delta-deg D`
with D between -20 and +20 deg and not 0; anything else is an argparse
error (exit 2) before anything connects. No gripper-project code is
used, only the SDK motor protocol.

Dry run first (connects, reads, moves nothing):

```bash
uv run python -m muse_qc.gripper_motor --delta-deg 10
```

It prints the six arm joints and the gripper reading through
`muse_qc/arm.py`, the plan (ramp out at 5 deg/s, hold 2 s, ramp back,
hold 1 s at 100 Hz) and the expectation: 0 deg = closed, more negative
= more open, about 0.32 mm total jaw opening per motor degree, so
+10 deg should close the jaws by about 3 mm. Then, with the hand on
the power supply switch and nothing between the jaws:

```bash
uv run python -m muse_qc.gripper_motor --delta-deg 10 --execute
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
human runs it, never CI and never an agent. `muse_qc/gripper_grip.py`
(checked by `tests/test_gripper_grip.py`, fake SDK and fake serial
only; the loop lives in the shared `muse_qc/gripper_stream.py`, which
also hosts the `GripperStream` moved over from
`muse_qc/gripper_motor.py` so that module stays at or below 300
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
uv run python -m muse_qc.gripper_grip --open-mm 40
```

Then, with the arm resting on its support and the hand on the power
supply switch:

```bash
uv run python -m muse_qc.gripper_grip --open-mm 40 --execute
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
measured it (`python -m muse_qc.vision_sam "white cube"` gives
table_mm A), the arm hovered at 100 mm over A
(`python -m muse_qc.hover_move --table-mm A --hold-s 30 --execute`),
the human slid the cube exactly under the centre between the finger
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

`muse_qc/xy_correction.py` (140 lines) models this in base polar
coordinates: real_angle = cmd_angle + YAW_DEG, real_r = cmd_r plus a
radial error R_ERR linear in cmd_r. YAW_DEG is the mean of the three
yaw errors; R_ERR is the least-squares line over the three radial
errors. Outside the fitted radius range R_ERR holds the end value
and never extrapolates. `correct_base_target_mm` inverts the model,
so commanding its output puts the real tip on the part. Offline fit
report (`python -m muse_qc.xy_correction`):

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
at a table position the human got from `python -m muse_qc.vision_sam`.
The arm hovers 100 mm over it, opens the gripper, lowers the tip
slowly while watching the finger sensors, closes until the left and
right fingers report contact, lifts the part to the hover height,
holds, puts it back down, opens, lifts and returns to rest. Only the
human runs it, never CI and never an agent.

Three modules share the work (`tests/test_pick.py`, fake SDK, fake
sensor and fake IK only): `muse_qc/pick_plan.py` builds the labelled
arm waypoint plan (trapezoid hover ramp at 15 deg/s with 30 deg/s2
plus a 0.5 s hover settle, 1 mm ladder fast at 40 mm/s above the
slow zone and 10 mm/s near the part, lift slow for the first 10 mm
then fast, put-down 2 mm higher, fast raise, trapezoid return ramp
plus 1 s settle, all at 100 Hz), `muse_qc/pick_loop.py` flies it in
one control loop, and `muse_qc/pick_test.py` is the CLI. The
trapezoid ramps start and stop smoothly instead of jumping to full
speed, and the 0.5 s hover settle lets the joints rest after the
ramp before the tip dives; both soften the visible tip swing, which
happens behind the motor encoders and no gain can see. The gripper
opens to `--open-mm` during the hover ramp (holding at hover until
fully open) instead of in a separate phase afterwards. Every arm
waypoint passes `safety.check_target` per cycle with the grasp floor
`GRASP_Z_MIN_M` = 4 mm on the lower/lift/put-down/raise waypoints
only; everywhere else the 30 mm floor is unchanged, and no safety
check was given up for the speedup. Any contact while lowering
stops the descent at once, holds 0.5 s, climbs back up the same
ladder and returns. `--no-grip` only lowers to `--grip-z-mm` and
holds, so the human can check the real tip height with a ruler
first. `--grip-z-mm` takes 5 to 60 mm (default 5), `--open-mm`
25 to 50 mm (default 28), `--hold-s` 0.5 to 30 s (default 1),
`--speed-deg-s` 2 to 15 deg/s for the free ramps (default 15);
anything else is an argparse error (exit 2) before anything
connects. `--no-xy-correction` flies the uncorrected target for A/B
tests, to measure the uncorrected error near the part. No
gripper-project code is used, only the serial line protocol and the
SDK motor protocol.

Order, with the hand on the power supply switch:

```bash
uv run python -m muse_qc.pick_test --table-mm X Y
uv run python -m muse_qc.pick_test --table-mm X Y --no-grip --grip-z-mm 40 --execute
uv run python -m muse_qc.pick_test --table-mm X Y --no-grip --grip-z-mm 20 --execute
uv run python -m muse_qc.pick_test --table-mm X Y --execute
```

The height correction is verified only at 100 mm so far, hence the
40 and 20 mm ruler stops before the grip run with one part. Parts
must lie 31 to 34.5 cm from the joint 1 axis (the verified x/y
correction range).

The default grip height is 5 mm: at 8 mm only the soft finger tips
touched the 14.5 mm high part and it slipped out while lifting, at
5 mm it held. The first pick used the default with the part placed
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

## 29. Find parts on the table (empty reference + SAM confirm)

SAM 3.1 on the full 1920x1080 frame finds the white 25 mm cube
reliably, but not the grey/orange connector parts (about 25 px wide,
low contrast on light wood): on 2026-10-29 the phrases "small part",
"connector", "terminal block", "electrical connector" and "object"
all returned 0 boxes on the zoomed 640 px crop, while "grey object"
confirmed the part. `muse_qc/find_parts.py` therefore works in two
steps: it compares the frame with a reference photo of the empty
table (arm in its rest pose) and takes every new blob as a candidate,
then sends a zoomed crop around each candidate to SAM 3.1 to confirm
it and refine the centre. Only the human runs it, never CI and never
an agent (it opens the real table camera and contacts the Meta Model
API over the network).

Order, from the project root:

```bash
uv run python -m muse_qc.find_parts --save-empty
uv run python -m muse_qc.find_parts --no-sam
uv run python -m muse_qc.find_parts
```

First store the reference with the table empty and the arm in its
rest pose: `--save-empty` takes one frame with
`camera_top.open_table_camera` and saves it as
`recordings/empty_table.png`, then prints the path and exits 0.
Then place the parts and run with `--no-sam` first (change
detection only, one line per candidate with blob centre, area and
table mm, ending in `candidates: M`), then with SAM (default phrase
`grey object`, overridable with `--phrase`; one SAM call per
candidate on a square crop of side max(3x blob side, 160 px),
upscaled to 640 px). Each line reports the centre used (SAM box
centre when confirmed, blob centre otherwise), the area, the table
position and the SAM verdict (`confirmed` with confidence, `no box`
or `skipped`), sorted by table x; the last lines give the
`parts: N confirmed of M candidates` count and the path of the
annotated copy at `recordings/parts_<YYYYmmdd_HHMMSS>.png` (blob
rectangles, green when confirmed, yellow otherwise, with id and
table mm). A missing reference or a size mismatch prints a short
message with exit 2; more than 12 blobs prints `too many changes`
with exit 1.

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
centre maps inside the table rectangle (0 to 990 mm in x, 0 to
540 mm in y). The table position uses the same calibration and
marker homography as `muse_qc/vision_sam.py`, and the key handling
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
planning error. `muse_qc/hold_test.py` measures it: it flies the
`hover_move` plan (100 mm over the corrected target, same safety
checks per cycle, same 5 deg/s ramp, return to rest and settle,
disable in finally), reads all six joints every cycle while
holding, prints per-joint mean error, peak-to-peak and RMS plus
the dominant swing frequency of joints 4 and 5 (zero crossings of
the error around its mean, `n/a` below 4 crossings), and saves the
samples as `recordings/hold_<YYYYmmdd_HHMMSS>.csv`. Only the human
runs it, never CI and never an agent. From the project root:

```bash
uv run python -m muse_qc.hold_test --table-mm X Y --hold-s 20
uv run python -m muse_qc.hold_test --table-mm X Y --hold-s 20 --gains soft --execute
```

`--hold-s` takes 5 to 60 s (default 20); anything else is an
argparse error (exit 2) before anything connects. The dry run
(default) connects, reads, checks every waypoint and moves
nothing. With `--execute` the human confirms with `yes`, then the
arm holds the hover pose for the given time.

`--gains soft` is the A/B candidate: `muse_qc/arm_gains.py` keeps
the position integral off (pos_ki 0.0) and lowers pos_kp by 30 %
for joints 4 and 5 only (70 to 49, 60 to 42; velocity loop
unchanged). The profile is printed joint by joint before the
`yes` prompt. A/B order: same point, first `--gains default`,
then `--gains soft`, and compare peak-to-peak and swing of joints
4 and 5; watch the mean error for sagging (without the integral
term the joint may rest slightly off target under load).

The soft gains live only in a runtime copy:
`write_runtime_yaml` copies the SDK hardware yaml into a
temporary directory (never into the repo, never into the SDK
package) and `make_arm` passes it as `RebotArm(hw_yaml=...)`;
`--gains` exists on `muse_qc/pick_test.py` too (default
`default`, same describe lines). The SDK file stays untouched,
and a normal run restores the default gains by itself: the SDK
writes vel_kp/vel_ki/pos_kp/pos_ki into the Damiao registers 25
to 28 on every `mode_pos_vel` call from the loaded yaml values.

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
