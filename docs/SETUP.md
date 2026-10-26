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
it runs the suite in `tests/` (currently `tests/test_arm.py`).

## 7. Trying arm.py on the real arm (by hand only)

This step touches hardware, so only the human runs it, never CI and
never an agent. `arm.py` only reads: it cannot enable, zero or move
the arm (enable, zero, mode and send commands are unreachable through
it), but `connect()` still opens the real bus. From the project root:

```bash
uv run python -c "
from arm import Arm
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
feedback, so `arm.py` never reports `0.0` as an angle. Check the bus
connection and the motor power, then retry.
