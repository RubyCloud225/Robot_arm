# ROADMAP — Robot Arm Pick-and-Place Simulator

**Version:** 0.1 | **Updated:** 2026-10-22  
Milestones are sequential. Completion requires test evidence, not just source code.

## P0 — Simulation foundation

**Objective:** a visible, stable simulated arm, ball and target with readable state and controllable joints.

- [ ] Initialise public repository, licence, package manifest and `.gitignore`.
- [ ] Select a licensed arm/gripper model and record its attribution, degrees of freedom, joint limits and actuation interface.
- [ ] Create MJCF scene with floor, table, arm, ball and goal marker.
- [ ] Validate model compilation, contacts and stable idle simulation.
- [ ] Implement simulator load/reset/step wrapper.
- [ ] Implement arm joint/gripper state reads and world-frame object positions.
- [ ] Command safe bounded motion; test joint limits and gripper open/close.
- [ ] Add an interactive viewer and headless smoke test.
- [ ] Add unit/integration tests, installation notes and pinned dependencies.

**Exit:** scene boots, ball responds to gravity, arm moves under explicit controls, observations are finite and reset works for the same seed.

## P1 — Scripted pick-and-place

**Objective:** one known initial condition completes a real contact-based transfer.

- [ ] Define reach, descend, grip, lift, transit, lower, release, verify states.
- [ ] Implement bounded motion controller and state transition tests.
- [ ] Confirm two-finger collision/contact and stable grasp.
- [ ] Implement target success predicate and failure modes.
- [ ] Record a full trajectory and reproducible episode result.

**Exit:** one documented seed completes a validated ball placement with no object teleportation or artificial attachment.

## P2 — Reliability and measurement

**Objective:** know where and why the scripted controller succeeds or fails.

- [ ] Sample reachable initial ball and target positions with fixed seeds.
- [ ] Run at least 100 held-out episodes.
- [ ] Report success rate, position error, episode duration and drop rate.
- [ ] Add regression cases for observed failure modes.
- [ ] Improve gripper/contact or controller tuning without changing success criteria retrospectively.

**Exit:** an evaluation report includes seed list, config, versions, metrics and a clear baseline success rate; no fixed success-rate percentage is presumed.

## P3 — Learning-ready environment

**Objective:** make the task trainable through a standard agent interface.

- [ ] Expose `reset()` / `step()` and observation/action spaces.
- [ ] Define rewards, episode bounds and separate evaluation success signal.
- [ ] Unit-test rewards, action bounds, reproducibility and truncation.
- [ ] Save/reload policy checkpoints and experiment settings.
- [ ] Choose a training algorithm and establish a small-budget smoke test.

**Exit:** an agent can run seeded episodes and train without breaking physics or environment contracts.

## P4 — Policy training and evaluation

**Objective:** train a controller and compare it fairly with the scripted baseline.

- [ ] Train on documented seeds and budget.
- [ ] Evaluate on unseen seeds without training updates.
- [ ] Compare success rate, completion time, placement error and drop frequency.
- [ ] Record learning curves and reproducible configurations.
- [ ] Document failures and avoid overclaiming if learned policy underperforms.

**Exit:** results can be reproduced from published configuration and evaluation scripts.

## P5 — Optional extensions

- [ ] Object size/mass/friction variation.
- [ ] Camera-based observations rather than privileged object coordinates.
- [ ] Multi-object sorting and different target positions.
- [ ] Additional supported robot models.
- [ ] Exploration of more advanced controllers and sample-efficiency methods.

**Exit:** each extension has its own documented experiment and evidence.

## Release gates

Public release should include correct licences/asset credits, install instructions tested on a declared platform, minimal demo, passing automated smoke tests, and a limitations section. Training results require recorded seeds, versions and a consistent evaluation protocol.
