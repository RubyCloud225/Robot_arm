from pathlib import Path

import pytest

from robot_arm_sim.cli import main


def test_robot_model_requires_robot_flag(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit, match="2"):
        main(["--robot-model", "/path/to/panda.xml"])

    assert "--robot-model can only be used with --robot" in capsys.readouterr().err


def test_robot_mode_reports_missing_model_path(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    missing_model = tmp_path / "panda.xml"

    with pytest.raises(SystemExit, match="2"):
        main(["--robot", "--robot-model", str(missing_model)])

    assert f"Panda model not found at {missing_model}" in capsys.readouterr().err
