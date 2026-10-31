#!/usr/bin/env bash
# Prepare the Seeed SDK clone for muse-qc-cell (docs/SETUP.md steps 1 to 3).
# Usage: bash scripts/setup_sdk.sh [target-folder]   (default: ../seeed-sdk)
# The SDK stays outside this repository; nothing is copied into it.
# Idempotent: safe to run again, it only fixes what is missing.
set -euo pipefail

PINNED_COMMIT="d4f3591bc0183de03f3ae8651298db0269aadfdd"
REPO_URL="https://github.com/Seeed-Projects/reBotArm_control_py.git"
TARGET="${1:-../seeed-sdk}"

# Step 1: clone at the pinned commit, or do nothing if already there.
if [ -e "$TARGET" ] && [ ! -d "$TARGET/.git" ]; then
  echo "sdk clone: $TARGET exists but is not a git checkout, aborting" >&2
  exit 1
fi
if [ ! -d "$TARGET/.git" ]; then
  echo "sdk clone: cloning $REPO_URL into $TARGET"
  git clone "$REPO_URL" "$TARGET"
fi
if [ "$(git -C "$TARGET" rev-parse HEAD)" = "$PINNED_COMMIT" ]; then
  echo "sdk clone: already at $PINNED_COMMIT, nothing to do"
else
  echo "sdk clone: checking out pinned commit $PINNED_COMMIT"
  git -C "$TARGET" fetch --quiet origin || true
  git -C "$TARGET" checkout --quiet "$PINNED_COMMIT"
fi

# Step 2: packaging workaround, appended only if not there yet.
WORKAROUND='include = ["reBotArm_control_py*"]'
if grep -Fq "$WORKAROUND" "$TARGET/pyproject.toml"; then
  echo "sdk packaging: workaround already present, nothing to do"
else
  printf '\n[tool.setuptools.packages.find]\n%s\n' "$WORKAROUND" >> "$TARGET/pyproject.toml"
  echo "sdk packaging: appended [tool.setuptools.packages.find] workaround"
fi

# Step 3: select the B601-DM hardware config, changed only if needed.
YAML="$TARGET/config/rebotarm.yaml"
WANT='hardware_yaml: "rebotarm_dm.yaml"'
if grep -q '^hardware_yaml:[[:space:]]*"rebotarm_dm.yaml"[[:space:]]*$' "$YAML"; then
  echo "sdk config: hardware_yaml already rebotarm_dm.yaml, nothing to do"
elif grep -q '^hardware_yaml:' "$YAML"; then
  sed -i "s|^hardware_yaml:.*|$WANT|" "$YAML"
  echo "sdk config: hardware_yaml set to rebotarm_dm.yaml"
else
  printf '%s\n' "$WANT" >> "$YAML"
  echo "sdk config: hardware_yaml set to rebotarm_dm.yaml"
fi
