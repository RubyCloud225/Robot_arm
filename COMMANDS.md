# Project Commands

Run commands from the repository root unless noted otherwise.

## Create an environment

Using `uv` with the committed lockfile:

```sh
uv sync --locked --extra dev
```

Or use a virtual environment and the exact-version pip requirements export:

```sh
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## Run the simulation

Open the interactive MuJoCo viewer:

```sh
python -m robot_arm_sim
```

The floor uses a contrasting checker pattern for depth and motion cues. Ground
is world z=0 with +Z pointing up; the table surface is z=0.50 m. The visible
floor patch is 4 × 4 m, while physical plane contact extends infinitely beyond
it. Floor friction is `(1, 0.005, 0.0001)` (sliding, torsional, rolling), with
three-dimensional contact constraints. Use the viewer's contact-point and
contact-force overlays to inspect dropped-ball contacts.

Run a headless physics check:

```sh
python -m robot_arm_sim --headless --steps 1000
```

Run with a custom TOML configuration; its model path is relative to that file:

```sh
python -m robot_arm_sim --config path/to/config.toml
```

The installed `robot-arm-sim` command and `python scripts/run_simulation.py`
provide the same runner.

### Run with the Franka Panda

The Panda model is not bundled or downloaded automatically. Place a locally
obtained, reviewed `panda.xml` and its referenced assets under
`.cache/robot_models/franka_emika_panda/` to use the default path, or specify
the XML path with `--robot-model`. See
[`docs/assets_and_licenses.md`](docs/assets_and_licenses.md) for the model
source and outstanding revision and asset-provenance review.

Launch the interactive Panda scene using the default model path:

```sh
python -m robot_arm_sim --robot
```

Launch using an explicit model path:

```sh
python -m robot_arm_sim --robot --robot-model /path/to/franka_emika_panda/panda.xml
```

Run the Panda scene headlessly for a fixed number of physics steps:

```sh
python -m robot_arm_sim --robot --headless --steps 1000
```

`--robot-model` requires `--robot`. The existing demo scene remains the default;
add `--headless --steps 1000` without `--robot` to run its headless check.

The robot is fixed on a pedestal at `(-0.65, 0, 0.50)` m, facing +X across
the table. Startup rejects reset poses that penetrate the scene supports.
See [robot placement](docs/robot_placement.md) for tested sweep bounds and
the command to enable real-model integration tests with `ROBOT_ARM_PANDA_XML`.

Use `.venv/bin/python` for this blocking interactive viewer. This launcher
was verified on macOS; `mjpython` returned an internal viewer exception. If
the package is not installed, prefix the command with `PYTHONPATH=src`.
Select the `overview` or `top_down` fixed camera in the viewer. See
[scene settings](docs/scene_settings.md) for lighting, physics rationale and
collision-filter verification.

## Test and verify

Run the test suite:

```sh
python -m pytest
```

Run only the robot-scene and CLI tests:

```sh
python -m pytest tests/test_simulation.py tests/test_cli.py
```

Check the dependency lock and project environment:

```sh
uv lock --check
uv sync --locked --extra dev
```

## Update dependencies

After changing dependencies in `pyproject.toml`, regenerate the lock and pip
requirements export:

```sh
uv lock
uv export --format requirements-txt --locked --extra dev --no-hashes --output-file requirements.txt
```
