# AGENTS.md — Robot_arm

## 1. Purpose

This document defines mandatory operating instructions for all AI coding agents working within the `Robot_arm` repository.

The repository is under direct human control. AI agents may assist with analysis, research, planning, code review and implementation only within limits explicitly authorised by the repository owner.

**Human approval takes precedence over agent autonomy.**

These instructions apply to all repository files, scripts, configurations, build systems, simulation assets, documentation and generated outputs.

## 2. Mandatory Planning Mode

Every new task MUST begin in **PLANNING MODE**. Agents MUST NOT begin implementation immediately after receiving a task.

Required workflow:

1. Read the relevant existing repository files.
2. Identify the requested objective and current implementation.
3. Identify dependencies, potential risks and affected components.
4. Present a proposed implementation plan.
5. Identify every file that would be created, modified, moved or deleted.
6. Present the proposed code or exact changes for review.
7. Request explicit approval from the repository owner.
8. Remain in planning mode until approval is granted.

Planning approval does not automatically authorise implementation unless explicitly stated. Silence, encouragement or acknowledgement is not approval.

## 3. Mandatory Code Approval

**ALL code requires explicit human approval BEFORE implementation.** This applies to new source files, existing code, scripts, build configurations, simulation models, tests, dependencies, refactoring, generated code, utilities and bug fixes.

Before implementation, agents MUST provide:

1. The exact files affected.
2. The reason for each proposed change.
3. Complete proposed code or an exact, reviewable diff.
4. Potential impact on existing functionality.
5. Proposed tests and validation procedures.

No code may be written, replaced, patched or deleted before approval. Approval covers only the changes presented; material deviations require renewed approval.

## 4. No Compilation or Execution Without Permission

**Agents MUST NOT compile, build, run or execute the project without explicit permission.**

Without separate authorisation, agents MUST NOT invoke compilers or build systems, execute project scripts, launch simulations, run tests or benchmarks, install dependencies, or run formatters, linters or generators that modify files.

Implementation approval DOES NOT imply permission to compile, run or test. Ask separately before any build, test or execution. Read-only file inspection and static reasoning are permitted.

## 5. Script and File Protection

Agents MUST NOT alter existing scripts without explicit approval. This includes shell and Python scripts, CMake files, Makefiles, CI/CD workflows, simulation launch scripts, test runners, environment setup and deployment scripts.

Do not silently rewrite, repair, reformat, rename, move, delete, chmod or otherwise modify scripts. Do not alter unrelated files. Every script change requires an exact proposed diff and explicit approval.

## 6. Repository Integrity

Preserve the existing repository structure and implementation unless instructed otherwise.

Without authorisation, do not delete or overwrite files, reset or revert changes, force-push, rewrite Git history, switch branches in a way that affects uncommitted work, commit or push changes, update submodules, alter dependencies or modify robotics assets.

Do not assume incomplete, failing or unused code is disposable.

## 7. Robotics and Simulation Safety

Without explicit approval, agents MUST NOT change robot joint definitions or limits, actuator mappings or ranges, gripper/end-effector geometry, contact/collision properties, MJCF or other model definitions, frames and coordinate conventions, control policies, simulation timesteps/solvers, safety limits or hardware interfaces.

Explain effects on robot behaviour, simulation validity and reproducibility. Distinguish verified facts from assumptions. **Do not equate actuator control values with joint angles unless verified.**

## 8. Code Style and Explanatory Comments

**Prefer readable, well-documented code with explanatory notes around the implementation.**

- Include concise comments adjacent to non-obvious logic explaining **what the code is doing and why the approach is used**.
- Document important assumptions, invariants, coordinate frames, units, dimensions, tolerances and safety constraints where relevant.
- For robotics and simulation code, annotate actuator-to-joint mappings, control ranges, frame conversions, contact assumptions and model-specific behaviours.
- Provide brief function-level documentation for substantive functions, explaining purpose, inputs, outputs, side effects and significant failure conditions.
- Add short section comments when they help readers follow multi-step algorithms, data flow or control flow.
- Keep comments technically accurate and update them when approved code changes make them stale.
- Avoid noisy line-by-line narration of obvious statements, redundant comments, speculative claims or excessive decorative comment banners.
- Match the language and documentation conventions of the surrounding code unless a different style is approved.

**Comments, docstrings and documentation are also file changes.** Do not add, rewrite or remove them without presenting the exact changes and obtaining approval first. A request to improve documentation is not blanket permission to edit source files.

## 9. No Unauthorised Background Actions

Do not initiate autonomous implementation workflows, launch subordinate coding agents with unrestricted write access, continue into implementation after presenting a plan, apply changes while requesting approval, run implicit validation, schedule automatic changes or use alternative tools to bypass these restrictions.

All delegated agents must follow this document.

## 10. Required Response Format

For each development task, present:

### Objective
What the task is intended to achieve.

### Current State
Facts verified from the repository.

### Proposed Approach
Implementation strategy and relevant alternatives.

### Files Affected
Every file proposed for creation, modification or deletion.

### Proposed Changes
Complete code or exact diffs for review, including proposed comments.

### Risks and Assumptions
Regressions, safety concerns, dependencies and unknowns.

### Validation Plan
Commands and checks proposed, **not executed**.

### Approval Required
Clearly list outstanding permissions, then stop.

## 11. Approval Boundaries

Approvals are explicit and task-specific:

- **PLAN APPROVED** — Design accepted; no file changes unless separately authorised.
- **CODE APPROVED** — Only the exact reviewed code changes may be implemented.
- **BUILD APPROVED** — Only specified build or compilation commands may run.
- **TEST APPROVED** — Only specified tests or simulation executions may run.
- **SCRIPT CHANGE APPROVED** — Only specifically reviewed script modifications may be applied.

Equivalent natural-language approvals are valid only when their scope is clear. One approval never implies the others. If ambiguous, stop and ask.

## 12. Mandatory Stop Conditions

Stop and request guidance if a change extends beyond approved scope; a script change, dependency, build or test becomes necessary but is not authorised; an unexpected error needs further edits; files contain unexpected modifications; implementation diverges from the approved design; or authorisation is unclear.

Do not independently work around these restrictions.

## 13. Final Operating Principle

**Inspect first. Plan second. Present the code and comments. Obtain approval. Implement only the approved changes. Compile and test only when separately authorised.**

The repository owner retains full control over all modifications and execution. Convenience, speed and autonomy never override these requirements.
