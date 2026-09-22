Fix gripper frame physics in OpenArm USD assets (authored masses + fixed joints)

## Summary

Two related asset bugs in `openarm_bimanual_physics.usd` and `openarm_unimanual_physics.usd`:

**Bug 1 — Incorrect mass properties.** The `*_hand` and `*_ee_tcp` rigid bodies have `PhysicsMassAPI` with only `diagonalInertia` authored and **no `physics:mass`**, so PhysX uses its default of **1.0 kg each**. In the upstream URDF (`openarm_description`, parallel-link gripper) `hand` is a real link of **0.127 kg** and `hand_tcp` (renamed `ee_tcp` in the USD) is a **massless TCP frame**. The extra ~1.9 kg per arm at the tip inflates the joint-space inertia by 2.5× (shoulder) to 9× (wrist).

**Bug 2 — Fixed joints exported as revolute.** `*_hand` and `*_ee_tcp_joint` are `PhysicsRevoluteJoint` with limits `[0, ~0]` — locked revolute joints standing in for the URDF's fixed joints. They add four spurious DOFs to the articulation (`openarm_right_hand`, `openarm_right_ee_tcp_joint`, … appear in `Articulation.joint_names`).

## Changes

| prim | before | after |
|---|---|---|
| `*_hand` | no mass authored (→ 1.0 kg) | mass 0.127 kg, CoM `(1e-6, 0, -0.000549)`, inertia `(1.656e-4, 1.747e-5, 1.735e-4)` from the URDF |
| `*_ee_tcp` | no mass authored (→ 1.0 kg) | mass 1e-3 kg — kept as a rigid body so existing `prim_path` (cabinet env `FrameTransformer`) and `body_names` references keep working |
| `*_hand`, `*_ee_tcp_joint` | `PhysicsRevoluteJoint`, limits [0, 0] | `PhysicsFixedJoint` |

`scripts/tools/fix_gripper_frame_physics.py` is the script used to produce the changes (kept for reproducibility; it can be re-run on a re-exported asset).

## Verification (Isaac Lab 2.3.2, Isaac Sim 5.1)

Loaded the modified `openarm_bimanual.usd` via `OPENARM_BI_CFG`:

| | before | after |
|---|---|---|
| `len(joint_names)` | 22 | **18** (7×2 arm + 4 finger) |
| `hand` / `ee_tcp` mass | 1.0 / 1.0 kg | 0.127 / 0.001 kg |
| `M[joint7, joint7]` (q = 0) | 0.0516 | **0.0054** kg·m² |
| `M[joint1, joint1]` | 1.081 | **0.434** kg·m² |
| 50 sim steps at q = 0 | — | stable, max |q| 7e-4 |

Cross-check against hardware: on our OpenArm v1.0 a torque-chirp identification of joint7 (Damiao DM4310) gives a total joint inertia of ≈0.006 kg·m² (link + reflected rotor 0.0017), consistent with the corrected asset and an order of magnitude below the shipped one. In our replay of a 20 s multisine holdout, the corrected model matches the real joint within 0.4° / 0.1 Nm RMS.

## Notes

- `ee_tcp` could alternatively be merged into `hand` (URDF importer `merge_fixed_joints`); I kept it as a body to avoid breaking `prim_path`/`body_names` users. Happy to switch if you prefer.
- The hosted copy on NVIDIA Nucleus (`Isaac/Robots/OpenArm/...`) is byte-identical to this repo's root layer; once merged, that asset needs a re-sync. Tracked in isaac-sim/IsaacLab#7938.
- The URDF gripper mass itself may be low: our measured joint7 gravity torque is 0.72 Nm vs 0.54 Nm from the URDF masses (+33 %), suggesting the gripper motor (DM4310, ~0.3 kg) is not included in `hand`. Out of scope here, but worth a look in `openarm_description`.

🤖 Generated with [Claude Code](https://claude.com/claude-code)
