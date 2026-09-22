[bug] OpenArm USD assets: `hand` / `ee_tcp` bodies have no authored mass (PhysX default 1.0 kg) and fixed joints exported as revolute

### Describe the bug

#### Bug 1 — Incorrect mass properties (`hand`, `ee_tcp`)

In the OpenArm assets (`{ISAAC_NUCLEUS_DIR}/Robots/OpenArm/openarm_bimanual/openarm_bimanual.usd`, added in #4089), the prims `openarm_{left,right}_hand` and `openarm_{left,right}_ee_tcp` have `PhysicsRigidBodyAPI` + `PhysicsMassAPI` applied but **no authored `physics:mass`**, so PhysX falls back to its default of **1.0 kg each** (inspected with `Usd.Stage.Open` on the hosted USD — `physics:mass` has no authored value on either prim).

Compared with the upstream OpenArm v1.0 URDF (`openarm_description`, parallel-link gripper):

| USD prim | URDF counterpart | URDF mass | USD effective mass |
|---|---|---|---|
| `openarm_*_hand` | `openarm_*_hand` (real link) | **0.127 kg** (`end_effector/parallel_link/config/inertials.yaml`) | 1.0 kg |
| `openarm_*_ee_tcp` | `openarm_*_hand_tcp` (massless TCP frame, `<link name=".."/>` with no inertial) | 0 | 1.0 kg |

So `hand` is a real body whose mass was not carried over (≈8× too heavy), and `ee_tcp` is a TCP frame that should be massless (or merged into its parent) but became a 1 kg rigid body.

#### Bug 2 — Fixed joints exported as revolute

Both are attached with **`PhysicsRevoluteJoint`** (`openarm_*_hand`, `openarm_*_ee_tcp_joint`), whereas the URDF uses fixed joints. They therefore appear as extra articulation DOFs (`openarm_right_hand`, `openarm_right_ee_tcp_joint` show up in `Articulation.joint_names`).

Effect on the joint-space inertia (diagonal of `root_physx_view.get_generalized_mass_matrices()`, right arm, q = 0):

| joint | as shipped | with hand = 0.127 kg, ee_tcp ≈ 0 | ratio |
|---|---|---|---|
| joint1 | 1.0807 | 0.434 | 2.5× |
| joint2 | 1.0799 | 0.433 | 2.5× |
| joint4 | 0.3526 | 0.098 | 3.6× |
| joint6 | 0.0520 | 0.0060 | 8.7× |
| joint7 | 0.0516 | 0.0056 | **9.2×** |

Cross-check against the physical robot (our measurements): a torque-chirp identification of the real joint7 (Damiao DM4310) gives a total joint inertia of ≈0.006 kg·m²; a sim-vs-real replay of a 20 s multisine holdout with the corrected masses matches within 0.4° / 0.1 Nm RMS, while the shipped asset is an order of magnitude off (residual plot attached). Controllers or policies tuned on this asset (e.g. the reach tasks / `OPENARM_BI_HIGH_PD_CFG` gains) see dynamics far from the real arm.

Expected:
- `openarm_*_hand`: author `physics:mass = 0.127` (and the inertia/CoM from the URDF).
- `openarm_*_ee_tcp`: massless — either drop the RigidBody/Mass APIs and keep it as a plain `Xform` frame, or merge it into `hand` (`merge_fixed_joints` in the URDF importer would do this).
- Both attachment joints should be **fixed**, not revolute.

### Steps to reproduce

```python
# ./isaaclab.sh -p repro.py --headless
import argparse, torch
from isaaclab.app import AppLauncher
ap = argparse.ArgumentParser(); AppLauncher.add_app_launcher_args(ap); args = ap.parse_args()
app = AppLauncher(args).app

from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab_assets.robots.openarm import OPENARM_BI_CFG

sim = SimulationContext(SimulationCfg(dt=0.002, device=args.device))
robot = Articulation(OPENARM_BI_CFG.replace(prim_path="/World/openarm")); sim.reset()

masses = robot.root_physx_view.get_masses()[0]
for name, m in zip(robot.body_names, masses):
    if name.endswith("_hand") or name.endswith("_ee_tcp"):
        print(f"{name:28s} mass = {m:.3f} kg")

M = robot.root_physx_view.get_generalized_mass_matrices()[0]
i = robot.joint_names.index("openarm_right_joint7")
print("M[joint7, joint7] =", float(M[i, i]))
app.close()
```

Output:

```
openarm_left_hand            mass = 1.000 kg
openarm_right_hand           mass = 1.000 kg
openarm_left_ee_tcp          mass = 1.000 kg
openarm_right_ee_tcp         mass = 1.000 kg
M[joint7, joint7] = 0.05162
```

Expected `M[joint7, joint7]` ≈ 0.0056 (obtained by setting hand = 0.127 kg and ee_tcp ≈ 0 with `root_physx_view.set_masses()` and re-reading the mass matrix).

### System Info

- Commit: 37ddf6268 (Isaac Lab 2.3.2)
- Isaac Sim Version: 5.1.0-rc.19+release.26219.9c81211b.gl580.178.04 (pip `isaacsim`)
- OS: Ubuntu 24.04.5 LTS
- GPU: NVIDIA GeForce RTX 5080 Laptop GPU (16 GB)
- CUDA: 12.8 (torch) / 13.0 (driver)
- GPU Driver: 580.178.04

### Additional context

- Physical reference: OpenArm v1.0 (Damiao DM4310 / DM4340 / DM8009 on CAN-FD). Joint7 identified on 2026-09-22 with constant-velocity sweeps (friction) and torque chirps 0.1–3.5 Hz (inertia); the rotor inertia and gear ratio were read back from the motor firmware (Inertia = 1.689e-5 kg·m², Gr = 10 → 0.00169 kg·m² reflected).
- #4089 notes that the OpenArm USD files were "currently under review" — this may be a good time to fix the export. The unimanual asset (`openarm_unimanual.usd`) has the same two defects (verified on the source layers; fixed in the PR below).
- Workaround until fixed:

```python
masses = robot.root_physx_view.get_masses().clone()
for i, b in enumerate(robot.body_names):
    if b.endswith("_hand") or b.endswith("_ee_tcp"):
        masses[0, i] = 0.127 if b.endswith("_hand") else 1e-4   # URDF hand mass; ee_tcp is a TCP frame
robot.root_physx_view.set_masses(masses, torch.arange(1, device="cpu"))
```

### Proposed fix

The USD source lives in `enactic/openarm_isaac_lab` (its root layer is byte-identical to the Nucleus copy), so I opened a PR there with the fix — authored URDF mass/inertia on `hand`, negligible mass on `ee_tcp`, and both joints converted to `PhysicsFixedJoint`: **enactic/openarm_isaac_lab#19** (https://github.com/enactic/openarm_isaac_lab/pull/19). Verified in Isaac Lab 2.3.2: 18 DOFs (was 22), `M[joint7, joint7]` 0.0516 → 0.0054 kg·m², stable stepping.

Once that lands, the hosted `Isaac/Robots/OpenArm/...` asset will need a re-sync on the Nucleus side.

### Checklist

- [x] I have checked that there is no similar issue in the repo (**required**)
- [x] I have checked that the issue is not in running Isaac Sim itself and is related to the repo

### Acceptance Criteria

- [ ] `openarm_{left,right}_hand` has an authored `physics:mass` matching the URDF (0.127 kg) in both `openarm_bimanual.usd` and `openarm_unimanual.usd`
- [ ] `openarm_{left,right}_ee_tcp` carries no meaningful mass (negligible mass, or merged into `hand`)
- [ ] `hand` / `ee_tcp` attachments are fixed joints (no extra DOFs in `joint_names`)
- [ ] `M[joint7, joint7]` at q = 0 is ≈ 0.0056 kg·m² for the bimanual asset (≈ 9× smaller than now)
