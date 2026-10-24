# P0-003 — Robot model selection, interfaces, and asset licences

**Project:** Open-source Robot Arm Simulator  
**Date reviewed:** 2026-10-22  
**Decision:** Use the **Franka Emika Panda** model from Google DeepMind's **MuJoCo Menagerie**, specifically `franka_emika_panda/panda.xml` (standard MuJoCo model, **not** `mjx_panda.xml`).  
**Review status:** Source-level assessment complete; source revision pin, full mesh provenance audit, and runtime verification remain open. **Do not mark P0-003 fully accepted yet.**

## P0-003.01 — Compare two integrated arm-and-gripper MJCF models

| Criterion | Franka Emika Panda (**selected**) | UFACTORY xArm7 (alternative) |
|---|---|---|
| Model directory | [`franka_emika_panda`](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda) | [`ufactory_xarm7`](https://github.com/google-deepmind/mujoco_menagerie/tree/main/ufactory_xarm7) |
| Main MJCF | `panda.xml` | `xarm7.xml` |
| Arm DoFs | 7 revolute | 7 revolute |
| Included gripper | Parallel-jaw, two sliding finger joints | Linked parallel-jaw gripper with driver/follower joints |
| Arm actuation | Seven affine general actuators acting as position-servo commands | Seven general actuators with configured position-style gains/biases |
| Gripper actuation | One tendon-driven general actuator, **0–255 control units** | One tendon-driven general actuator, **0–255 control units** |
| Coupling | Fixed `split` tendon plus joint equality constraint | Fixed `split` tendon, joint equality and linkage constraints |
| Contacts | Robot collision meshes and explicit fingertip pad boxes; base/first-link exclusion | Finger-pad collision boxes; several gripper linkage contact exclusions |
| Project licence | Apache-2.0 in model subdirectory | BSD-3-Clause in model subdirectory |
| Relative integration difficulty | Lower; simple two-finger mechanism, clear home keyframe | Higher; linked gripper with more joints and constraints |

**Rationale:** The Panda provides the simplest complete, openly documented combination of a seven-axis arm, parallel gripper, home state, and explicit fingertip collision geometry. The xArm7 is a credible fallback if Panda dynamics or contact performance prove unsuitable.

**Note:** These are source-level MJCF assessments, **not** claims that either model has been loaded or tested in this project.

## P0-003.02 — Selection, source, author, revision, licence

- **Name:** Franka Emika Panda (standard MuJoCo Menagerie model).
- **Hardware origin:** Franka Emika; MJCF adapted from upstream Franka robot descriptions.
- **MJCF curator/publisher:** Google DeepMind / MuJoCo Menagerie contributors.
- **Menagerie attribution:** Kevin Zakka, Yuval Tassa, MuJoCo Menagerie Contributors (see Menagerie README/CITATION.cff).
- **Main source:** https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda
- **Definition:** https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_emika_panda/panda.xml
- **Model readme:** https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_emika_panda/README.md
- **Licence:** https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_emika_panda/LICENSE — Apache-2.0 model licence, subject to asset provenance verification below.
- **Upstream revision:** **NOT YET PINNED.** Source examined on 2026-10-22 at `main`, which is mutable. Before downloading or vendoring, resolve a 40-character commit SHA and record it in `docs/robot_model.lock` (or `assets_and_licenses.md`); never treat `main` as a reproducible pin.
- **Release:** No release tag independently validated; prefer commit SHA.

## P0-003.03 — Mesh, texture and derivative asset rights

The model README describes conversion of Franka's original DAE/URDF data into OBJ, conversion of collision meshes, and decomposition of `link5` geometry. Its model-directory licence is Apache-2.0, **but a licence heading is not sufficient by itself to establish the rights of every upstream-derived mesh.**

- [x] Identify the model-directory Apache-2.0 licence and model derivation notes.
- [x] Identify that `panda.xml` loads `assets/` OBJ/STL meshes and uses inline material colours.
- [ ] At pinned revision, enumerate every referenced `mesh file=` and `texture file=` (including recursively included XML) and its exact path.
- [ ] Review the original Franka `franka_description` licence and attribution for both visual and collision source meshes; retain origin URLs and relevant notices.
- [ ] Check whether converted/decomposed meshes introduce additional licence or attribution conditions.
- [ ] Confirm any standalone texture/skybox assets used by `scene.xml` and their terms; avoid copying arbitrary upstream demo scenery without review.
- [ ] Record file hashes and source-to-derived mapping in an asset manifest.
- [ ] Include upstream `LICENSE`, any required `NOTICE`, author credits and modifications if redistribution is chosen.

**Distribution gate:** Until source-mesh redistribution rights are confirmed, do **not** commit copied meshes or textures to the public repository. A pinned external fetch with licence review is the initial preference.

## P0-003.04 — Arm joint mapping (standard `panda.xml`)

All seven arm joints are MuJoCo **hinge** (revolute) joints. The default joint axis in the Panda class is local `(0, 0, 1)`; joint-frame rotations mean these are **local**, not global, axes. Ranges are in **radians**. Home state is read from the named `home` keyframe, **not** inferred from geometric zero.

| Joint | Joint type | Local axis | Range (rad) | Home qpos (rad) |
|---|---|---|---|---:|
| `joint1` | hinge | `(0,0,1)` | `[-2.8973, 2.8973]` | 0 |
| `joint2` | hinge | `(0,0,1)` | `[-1.7628, 1.7628]` | 0 |
| `joint3` | hinge | `(0,0,1)` | `[-2.8973, 2.8973]` | 0 |
| `joint4` | hinge | `(0,0,1)` | `[-3.0718, -0.0698]` | -1.57079 |
| `joint5` | hinge | `(0,0,1)` | `[-2.8973, 2.8973]` | 0 |
| `joint6` | hinge | `(0,0,1)` | `[-0.0175, 3.7525]` | 1.57079 |
| `joint7` | hinge | `(0,0,1)` | `[-2.8973, 2.8973]` | -0.7853 |

Source: `panda.xml` default `<joint>` class, joint overrides, and `<key name="home">`. **Confirm resolved compiled axes and limits at runtime** before implementing the control adapter.

## P0-003.05 — Gripper mapping

- Bodies: `hand`, `left_finger`, `right_finger`.
- Joints: `finger_joint1` (left), `finger_joint2` (right).
- Joint types: **slide**, inherited from class `finger`.
- Local joint axes: `(0,1,0)` in the respective finger frames; `right_finger` is rotated, yielding opposing motion.
- Per-finger joint limits: `0` to `0.04` **metres**.
- Approximate free finger separation from joint travel: **0–0.08 m** (nominal, not guaranteed graspable ball diameter or measured inner-surface spacing).
- `split` fixed tendon: coefficients `0.5` for each finger.
- Equality constraint: `finger_joint1` tracks `finger_joint2`.
- Gripper control actuator: `actuator8`, control range **0–255 arbitrary actuator command units**, with 255 corresponding to the open home control in the upstream keyframe.
- Home finger qpos: `0.04`, `0.04` m.

**Important:** Never set `actuator8` to a requested width in metres. Derive and verify a conversion with tests and inspect actual joint positions; the source contains an affine gain and bias, not a direct-width actuator.

## P0-003.06 — Actuator mapping and units

The standard Panda model uses **`general`** actuators (not MuJoCo `<position>` shortcuts), with affine position-servo-like gains/biases. For the seven arm channels, commands are **target positions in radians under the supplied MJCF parameters**, not raw torque controls. Actual achieved angles depend on dynamics and actuator saturation.

| Actuator | Transmission | Command kind | `ctrlrange` | Notes |
|---|---|---|---|---|
| `actuator1` | `joint1` | joint position setpoint, rad | `[-2.8973, 2.8973]` | inherited class limit |
| `actuator2` | `joint2` | joint position setpoint, rad | `[-1.7628, 1.7628]` | override |
| `actuator3` | `joint3` | joint position setpoint, rad | `[-2.8973, 2.8973]` | inherited |
| `actuator4` | `joint4` | joint position setpoint, rad | `[-3.0718, -0.0698]` | override |
| `actuator5` | `joint5` | joint position setpoint, rad | `[-2.8973, 2.8973]` | inherited; smaller force limit |
| `actuator6` | `joint6` | joint position setpoint, rad | `[-0.0175, 3.7525]` | override; smaller force limit |
| `actuator7` | `joint7` | joint position setpoint, rad | `[-2.8973, 2.8973]` | inherited; smaller force limit |
| `actuator8` | `split` tendon | gripper-specific 0–255 command | `[0,255]` | **not radians or metres** |

Actuator forces have `forcerange` in upstream MJCF; arm channels inherit `[-87,87]` or override to `[-12,12]` (actuators 5–7). The gripper has `[-100,100]`. These are generalized actuator-force limits, **not interchangeable with joint torque limits without transmission analysis**.

Acceptance requires reading compiled `model.actuator_ctrlrange`, `model.actuator_trntype`, `model.actuator_trnid`, `model.jnt_range`, and joint addresses in a smoke test; commands must never depend on assumed positional ordering alone.

## P0-003.07 — End-effector and finger geometry/site

- Named bodies for observations: `hand`, `left_finger`, `right_finger`.
- Visual geometry uses `hand_*`, `finger_*` meshes (many visual mesh geoms are unnamed).
- The standard `panda.xml` does **not** define a named TCP/end-effector site in the inspected source; the MJX variant is different. **Do not reference an imaginary `grasp_site` or `ee_site`.**
- Project action: create a project-owned named site such as `grasp_site` under the `hand` body after confirming its transform and approach orientation; document its local transform and use `data.site_xpos`/`data.site_xmat` for observations.
- Finger collision pads are defined through five small box geometry defaults on each finger; these are largely unnamed in `panda.xml`. Identify and, if necessary, give project-owned names for reliable contact queries.

## P0-003.08 — Collision geometry and exclusions

- Collision arm geoms include `link0_c`, `link1_c`, ..., `link7_c`; `link5` has multiple convex-decomposition mesh geoms.
- The `hand` has `hand_c` collision geometry.
- Each finger includes a collision mesh and multiple fingertip pad collision boxes.
- Visual-only geoms use `contype=0` and `conaffinity=0` (they do not generate contacts).
- Source explicitly excludes contact between `link0` and `link1` in the robot XML. The model README also describes link7/link8 exclusion during derivation; **the standard current XML must be treated as authoritative**, not the history note.
- Some adjacent parent-child contacts may additionally be filtered by engine rules; verify compiled contact behaviour instead of presuming every mesh pair contacts.

**Grasping test gate:** create a ball collider small enough to fit within 0.08 m nominal jaw travel, verify both pad-to-ball contacts, friction and no tunnelling, and inspect `data.contact` under deterministic headless stepping. This is a later scene/physics acceptance test, not yet passed.

## P0-003.09 — Reproducible asset strategy

**Decision: fetch a pinned upstream model snapshot at install/setup time; do not vendor any copied upstream meshes until provenance review is complete.**

1. Resolve and record one exact 40-character Menagerie commit SHA.
2. Download that revision explicitly, *not* `main`, `master`, or latest release.
3. Extract **only** `franka_emika_panda/` with its required dependencies into a predictable ignored cache path such as `.cache/robot_models/franka_emika_panda/`.
4. Validate file SHA-256 checksums against a checked-in manifest created from the pinned snapshot.
5. Store/copy applicable model `LICENSE` and notices alongside cached assets.
6. Provide actionable offline instructions when installation cannot fetch upstream data.
7. CI should use the same locked revision and validate the same hashes.
8. Fail clearly on revision drift, missing meshes, changed file hashes or license mismatch; never silently retry against `main`.

Suggested lock-file format (populate *after* independently resolving a real SHA):

```yaml
model: franka_emika_panda
repository: https://github.com/google-deepmind/mujoco_menagerie
revision: REQUIRED_40_CHARACTER_COMMIT_SHA
entrypoint: franka_emika_panda/panda.xml
license: Apache-2.0
asset_manifest: docs/model_asset_manifest.sha256
```

## P0-003.10 — Mapping and acceptance status

This file is the canonical source-model mapping. Before P0-003 is closed, attach an executable model-introspection report with resolved joint and actuator definitions, a screenshot or headless test log showing initial load, an asset hash manifest, and verified upstream commit.

| Ticket | State | Evidence / remaining gate |
|---|---|---|
| P0-003.01 | **Source-reviewed** | Panda vs xArm7 compared; no runtime test |
| P0-003.02 | **Partial** | Model/source/credit/licence identified; SHA pin outstanding |
| P0-003.03 | **Partial** | Model licence found; independent mesh provenance audit outstanding |
| P0-003.04 | **Source-reviewed** | Joint table captured; compiled-model check pending |
| P0-003.05 | **Source-reviewed** | Finger/tendon constraints and ranges captured; opening measurement pending |
| P0-003.06 | **Source-reviewed** | Actuator controls and units mapped; compiled check pending |
| P0-003.07 | **Source-reviewed** | Bodies/geoms identified; custom TCP site not yet created |
| P0-003.08 | **Source-reviewed** | Contact geoms/exclusion identified; contact test pending |
| P0-003.09 | **Decision recorded** | Pinned external fetch chosen; pin/hash installer not yet implemented |
| P0-003.10 | **Document created** | This file; update after validation |

### Primary evidence sources

1. https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_emika_panda/panda.xml
2. https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_emika_panda/README.md
3. https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_emika_panda/LICENSE
4. https://github.com/google-deepmind/mujoco_menagerie/blob/main/ufactory_xarm7/xarm7.xml
5. https://github.com/google-deepmind/mujoco_menagerie/blob/main/ufactory_xarm7/README.md
6. https://github.com/google-deepmind/mujoco_menagerie/blob/main/ufactory_xarm7/LICENSE
7. https://github.com/google-deepmind/mujoco_menagerie/blob/main/README.md

*This is an engineering source review, not a legal opinion or proof of simulation execution.*
