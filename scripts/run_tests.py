#!/usr/bin/env python3
"""Run all repository tests using the local virtual environment."""

import os
from pathlib import Path
import subprocess
import sys


def main() -> int:
    """Run pytest, returning its exit status or an environment error."""
    root = Path(__file__).resolve().parents[1]
    python = root / ".venv" / "bin" / "python"

    if not python.is_file():
        print(
            f"Virtual environment interpreter not found: {python}",
            file=sys.stderr,
        )
        return 1

    # Make the src-layout package importable without an editable install.
    env = os.environ.copy()
    # Use the local Panda model unless another location was supplied.
    env.setdefault(
        "ROBOT_ARM_PANDA_XML",
        str(root / ".cache" / "robot_models" / "franka_emika_panda" / "panda.xml"),
    )
    # Resolve relative overrides against the same root used to run pytest.
    panda_path = Path(env["ROBOT_ARM_PANDA_XML"]).expanduser()
    if not panda_path.is_absolute():
        panda_path = root / panda_path
    if not panda_path.is_file():
        print(
            f"Panda model not found: {panda_path}\n"
            "Place panda.xml and its referenced assets at this location, "
            "or set ROBOT_ARM_PANDA_XML to an existing model file.",
            file=sys.stderr,
        )
        return 1
    env["ROBOT_ARM_PANDA_XML"] = str(panda_path)

    paths = [str(root / "src")]
    if env.get("PYTHONPATH"):
        paths.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = os.pathsep.join(paths)

    result = subprocess.run(
        [str(python), "-m", "pytest", "-v", "tests/", *sys.argv[1:]],
        cwd=root,
        env=env,
        check=False,
    )
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
