# Dev Log

## 2026-10-26

- Set up accounts, environment, repository, and Muse Code.

## 2026-10-27

- Documentation only: created docs/PLAN.md (goal, setup, schedule), extended AGENTS.md
  with agent-loop, evidence, offline-test and no-legacy rules, and
  declared the gripper project plus workshop repo in
  docs/THIRD_PARTY.md. No code changed, nothing committed.
- Built safety.py (pure pre-move checks: workspace box with 5 cm
  margin, tool z, SDK URDF joint limits with refuse-all fallback,
  joint 6 cable limit around user-set J6_REF, speed cap, step limits,
  200 ms state freshness, latched STOP, 4-sensor gripper rules) with
  tests/test_safety.py (33 tests, all green) and filled in the gripper
  entry in docs/THIRD_PARTY.md. No hardware touched, nothing committed.
- Fixed safety.py strict targets: every target needs full x/y/z position
  and all six joints ("incomplete-target" otherwise), joint 6 always
  checked against J6_REF; tests updated. No hardware touched, nothing committed.
- Documentation only: shortened docs/PLAN.md, updated AGENTS.md stack and
  layout plus ground rules, updated docs/THIRD_PARTY.md and docs/SETUP.md.
  No code changed, nothing committed.
- Made tests/test_safety.py SDK-independent (fake joint limits via fixture,
  SDK test skips without reBotArm_control_py), added README Contributions
  section, fixed AGENTS.md layout (inspection.py, tools/) and stack wording.
  safety.py unchanged. Nothing committed.
- Fixed review findings: safety.py refuses non-finite current x/y/z
  ("invalid-current-position"), requires 0 < speed, derives WORK_X_MAX_M
  and WORK_Y_MAX_M from TABLE_* minus EDGE_MARGIN_M (values unchanged)
  and notes the holding-lowering rule still needs real-gripper
  verification; preflight.py checks every command-line element plus the
  -m module, drops the unused constant and falls back from a failed fuser
  probe to lsof; arm.py and diagnose_joints.py warn about motor
  disable/sag on disconnect; tests added for every fix, full suite green.
  No hardware touched, nothing committed.
- Fixed docs (PLAN wording, full test list with SDK-independent note in
  SETUP, planned markers and line rule in AGENTS, planned gripper plus
  Requirements in README and THIRD_PARTY) and safety.py right-side limit
  (WORK_Y_MIN_M with both-side tests); full suite green (98 passed).
  No hardware touched, nothing committed.
