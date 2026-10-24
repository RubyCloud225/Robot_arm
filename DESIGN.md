# DESIGN — Robot Arm Pick-and-Place Simulator

**Status:** Design draft 0.1  
**Last updated:** 2026-10-22

## 1. Purpose

Implement an isolated, reproducible physics simulation of one robotic arm that picks up a ball, transports it over a table and places it at a designated target. Support incremental learning of simulation, control, grasping and policy training. All interfaces are independent of any downstream platform or product.

## 2. Design principles

1. **Physics first:** contacts, gravity, friction and joint limits come from MuJoCo rather than hard-coded animations.
2. **Reproducible:** fixed seeds, captured configurations and traceable evaluation results.
3. **Small, testable modules:** separate simulation, control, task scoring and training.
4. **Baseline before learning:** a deterministic controller must first demonstrate that the scene and task are feasible.
5. **Public by design:** no proprietary dependencies, hidden training data or unlicensed robot assets.
6. **Honest status:** no capability is considered implemented until executable tests support it.

## 3. Task definition

### Scene

- Static table and floor; gravity enabled.
- One fixed-base articulated arm, ideally 6–7 DoF, with parallel gripper.
- One movable **sphere** with a free joint, explicit mass, radius, friction and contact parameters.
- One target marker within reach and on a stable support surface.
- Fixed or seeded camera views for debugging, not required as policy inputs initially.

Choose concrete scene units in SI: metres, kilograms and seconds. Record the exact robot model, asset source, licence, joint limits and actuator semantics when P0 is complete.

### Episode

Reset arm, gripper, ball and target to valid positions; remove residual velocities; settle physics. Then:

1. **Approach:** align gripper above ball.
2. **Descend:** position fingers on opposite sides of ball.
3. **Grasp:** close gripper while observing contacts.
4. **Lift:** raise ball beyond a configurable clearance (initial target: 5 cm above its resting centre height).
5. **Transport:** move to target while maintaining the grasp.
6. **Place:** lower and open gripper.
7. **Verify:** ball stays within 5 cm of target centre on the support for 1 second.

The result is a failure if the ball falls from the table, the motion exceeds safety limits, a control error occurs or the episode time limit expires. A momentary pass through the target is **not** success.

### Initial episode parameters

| Parameter | Draft setting | Notes |
|---|---:|---|
| Physics step | 0.002 s | Tune after confirming model stability |
| Controller period | 0.04 s | 25 Hz control; 20 physics steps per action |
| Episode time limit | 20 s | Configurable |
| Placement radius | 0.05 m | Horizontal distance on target surface |
| Settle interval | 1.0 s | Ball must remain placed after release |
| Seed | Configurable integer | Capture each evaluated seed |

All thresholds are initial design values, not validated performance claims.

## 4. Logical architecture

```text
 Experiment configuration / seed
                |
                v
        Episode environment <---------- Evaluator / logger
        | reset()  step() |                       ^
        v                 v                       |
   Task scoring       Action checks ------------+
        ^                 |
        |                 v
   Observations <---- MuJoCo simulation
        ^                 ^
        |                 |
        +---- Controller / training policy
```

### Components

- **Simulation adapter:** owns MuJoCo model/data, loads MJCF, resets and steps physics; supplies model IDs and time state.
- **Observation builder:** constructs a consistent schema from joint position/velocity, gripper pose, ball pose/velocity, target pose, finger contacts and previous action. All coordinates and units are documented.
- **Action adapter:** accepts bounded end-effector displacement and gripper open/close command; converts commands through the selected robot's verified control method. Inverse kinematics or mocap control is a separate implementation decision, not presumed available in an arbitrary model.
- **Task environment:** controls episode reset, state transitions, rewards and termination/truncation.
- **Scripted controller:** deterministic state machine for approach, descend, grasp, lift, transport, place and verify; checks timeouts and transition conditions.
- **Trainer:** later uses a compatible environment API, saved configurations and separate evaluation seeds.
- **Evaluator/logger:** records success rate, completion time, distances, drop events, constraint violations and stepwise traces.

## 5. Core interfaces (conceptual)

```python
observation, info = environment.reset(seed=42)
observation, reward, terminated, truncated, info = environment.step(action)
```

Action draft: `[dx, dy, dz, gripper]` expressed as bounded values in `[-1, 1]` and mapped to capped Cartesian motion and gripper commands. This is a **design proposal**, not a implemented interface. Additional wrist control may be necessary depending on arm geometry.

Observation draft:

```text
joint_positions[N], joint_velocities[N]
gripper_position[3], gripper_opening[1]
ball_position[3], ball_velocity[3]
target_position[3]
left_contact[1], right_contact[1]
previous_action[4]
```

Log the world frame, finger frames and any transformations explicitly. Contact flags may be derived from MuJoCo's contact data and must be verified rather than inferred from gripper closure.

## 6. Control and training

**Phase A: scripted baseline.** Start with known ball and target locations. Validate each motion segment separately. Require actual two-finger contact and stable lifting, not teleportation or ball attachment.

**Phase B: variable placements.** Seed randomized reachable positions, test stability across independent runs and measure controller failures.

**Phase C: learned controller.** Expose the same task through a training-compatible API. Begin with a simple algorithm selected after observing state/action behavior (e.g. an off-policy continuous-action method). Use a sparse success metric alongside a documented shaped training reward if needed; do not evaluate training solely on shaped reward.

**Phase D: generalisation checks.** Compare performance on held-out seeds and modest object/physics variations. Avoid claiming robustness beyond tested ranges.

## 7. Rewards and measurements

Potential shaped reward components: reduced end-effector-to-ball distance, reliable grasp, achieved lift height, ball-to-target distance, and final successful placement. Penalise excessive actions or dropped objects only if the penalty definition is documented. Freeze task success separately from shaping to prevent reward gaming.

Evaluation outputs: `episode_id`, `seed`, robot/model version, config hash, controller version, success flag, final placement error (m), elapsed time (s), dropped flag, episode steps and termination reason. Store per-step trajectories when debugging; keep generated output outside source control.

## 8. Verification and quality

- **Unit:** observation dimensions, action clipping, reset seeding, transforms, reward math, success predicate and timeout handling.
- **Integration:** scene loads, gravity/contact enabled, robot stays within limits and reset restores deterministic state.
- **Scenario:** gripper opens/closes, ball is not artificially fixed, lift is physically supported, release causes correct physics.
- **Regression:** same model/config/seed yields equivalent outcomes within a specified numeric tolerance on a pinned software/hardware stack.
- **Performance:** time per simulated second and evaluations across at least 100 held-out seeded episodes after P2.

Do not promise bitwise determinism across platforms. Include CPU architecture, OS, MuJoCo version and settings in reproducibility reports.

## 9. Safety and limitations

This simulation does not model every real-world property (sensor noise, actuator delay, wear, deformable objects, calibration or emergency stops). Simulation success must not be presented as real-world robot readiness. An arm model and gripper must be assessed for realistic contact behavior before reported learning results are considered credible.

## 10. Open design decisions

- Which redistributable arm and gripper model to use.
- Whether initial control uses inverse kinematics, mocap-based tracking or joint-space targets.
- Exact collision meshes and friction coefficients for reliable sphere grasping.
- Which training algorithm and experiment budget to adopt after baseline evidence.
- Supported runtime platforms and minimum rendering configuration.
