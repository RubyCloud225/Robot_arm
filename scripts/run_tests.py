#!/usr/bin/env python3
"""Run repository tests and append their output to docs/test_runs.md."""

from datetime import datetime
import os
from pathlib import Path
import subprocess
import sys


def main() -> int:
    """Run pytest, record its output, and return its exit status.

    Return 1 if environment validation or writing the record fails.
    """
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

    started_at = datetime.now().astimezone().isoformat(timespec="seconds")
    process = subprocess.Popen(
        [str(python), "-m", "pytest", "-v", "tests/", *sys.argv[1:]],
        cwd=root,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    # Merge both output streams so failures appear in the saved transcript.
    output_lines: list[str] = []
    assert process.stdout is not None
    with process.stdout:
        for line in process.stdout:
            print(line, end="", flush=True)
            output_lines.append(line)
    returncode = process.wait()

    # Indent the transcript so test output cannot close a Markdown code fence.
    transcript = "".join(
        f"    {line}" for line in "".join(output_lines).splitlines(keepends=True)
    )
    record_path = root / "docs" / "test_runs.md"
    try:
        with record_path.open("a", encoding="utf-8") as record:
            record.write(
                f"\n## Test run: {started_at}\n\n"
                f"Exit status: {returncode}\n\n"
                f"{transcript}\n"
            )
    except OSError as error:
        print(f"Could not write test record: {error}", file=sys.stderr)
        return returncode if returncode != 0 else 1

    print(f"\nTest record appended to {record_path}")
    return returncode


if __name__ == "__main__":
    raise SystemExit(main())
