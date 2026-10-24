
"""P0-005.01 — Model loader regression tests.

All test models are generated in pytest-managed temporary directories.
No production model files or project scripts are modified.
"""

from pathlib import Path

import mujoco
import pytest

from robot_arm_sim.model_loader import ModelLoadError, load_model


VALID_MODEL = """
<mujoco model="loader_fixture">
  <option timestep="0.002"/>
  <worldbody>
    <body name="test_body" pos="0 0 1">
      <joint name="test_joint" type="hinge"
             axis="0 1 0" limited="true"
             range="-1 1"/>
      <geom name="test_geom" type="box"
            size="0.1 0.1 0.1" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="test_motor" joint="test_joint"
           ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"""


def write_model(tmp_path: Path, content: str) -> Path:
    """Create an isolated XML fixture for one loader test."""
    path = tmp_path / "model.xml"
    path.write_text(content, encoding="utf-8")
    return path


def test_valid_model_loads(tmp_path: Path) -> None:
    path = write_model(tmp_path, VALID_MODEL)

    model, data = load_model(path)

    assert isinstance(model, mujoco.MjModel)
    assert isinstance(data, mujoco.MjData)
    assert model.njnt == 1
    assert model.nu == 1
    assert model.opt.timestep == pytest.approx(0.002)
    assert data.time == pytest.approx(0.0)


def test_string_path_is_supported(tmp_path: Path) -> None:
    path = write_model(tmp_path, VALID_MODEL)

    model, data = load_model(str(path))

    assert model.njnt == 1
    assert data.time == 0.0


def test_relative_path_is_supported(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    write_model(tmp_path, VALID_MODEL)
    monkeypatch.chdir(tmp_path)

    model, _ = load_model("model.xml")

    assert model.njnt == 1


def test_missing_file_has_useful_error(tmp_path: Path) -> None:
    path = tmp_path / "does_not_exist.xml"

    with pytest.raises(ModelLoadError) as exc:
        load_model(path)

    assert "does_not_exist.xml" in str(exc.value)
    assert "resolve model path" in str(exc.value)


def test_directory_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(
        ModelLoadError, match="not a regular file"
    ):
        load_model(tmp_path)


@pytest.mark.parametrize("path", ["", " ", "\t", "\n"])
def test_empty_paths_are_rejected(path: str) -> None:
    with pytest.raises(ModelLoadError, match="cannot be empty"):
        load_model(path)


@pytest.mark.parametrize("path", [None, 123, 1.5, [], {}])
def test_invalid_path_types_are_rejected(path: object) -> None:
    with pytest.raises(TypeError, match="Model path must"):
        load_model(path)  # type: ignore[arg-type]


def test_malformed_xml_is_rejected(tmp_path: Path) -> None:
    path = write_model(
        tmp_path,
        "<mujoco><worldbody></mujoco>",
    )

    with pytest.raises(ModelLoadError) as exc:
        load_model(path)

    assert "Failed to load or compile" in str(exc.value)
    assert "model.xml" in str(exc.value)


def test_empty_file_is_rejected(tmp_path: Path) -> None:
    path = write_model(tmp_path, "")

    with pytest.raises(ModelLoadError):
        load_model(path)


def test_invalid_mujoco_element_is_rejected(
    tmp_path: Path,
) -> None:
    path = write_model(
        tmp_path,
        "<mujoco><invalid_element/></mujoco>",
    )

    with pytest.raises(ModelLoadError):
        load_model(path)


def test_unknown_joint_reference_is_rejected(
    tmp_path: Path,
) -> None:
    invalid = VALID_MODEL.replace(
        'joint="test_joint"',
        'joint="missing_joint"',
    )
    path = write_model(tmp_path, invalid)

    with pytest.raises(ModelLoadError) as exc:
        load_model(path)

    assert "missing_joint" in str(exc.value)


def test_missing_mesh_dependency_is_rejected(
    tmp_path: Path,
) -> None:
    path = write_model(
        tmp_path,
        """
        <mujoco>
          <asset>
            <mesh name="missing"
                  file="not_present.stl"/>
          </asset>
          <worldbody>
            <geom type="mesh" mesh="missing"/>
          </worldbody>
        </mujoco>
        """,
    )

    with pytest.raises(ModelLoadError) as exc:
        load_model(path)

    assert "not_present.stl" in str(exc.value)


def test_model_loading_preserves_original_error(
    tmp_path: Path,
) -> None:
    path = write_model(
        tmp_path,
        "<mujoco><bad/></mujoco>",
    )

    with pytest.raises(ModelLoadError) as exc:
        load_model(path)

    # Exception chaining preserves the upstream diagnostic for debugging.
    assert exc.value.__cause__ is not None


def test_model_file_is_not_modified(tmp_path: Path) -> None:
    path = write_model(tmp_path, VALID_MODEL)
    before = path.read_bytes()

    load_model(path)

    assert path.read_bytes() == before


def test_loading_produces_independent_states(
    tmp_path: Path,
) -> None:
    path = write_model(tmp_path, VALID_MODEL)

    model_a, data_a = load_model(path)
    model_b, data_b = load_model(path)

    assert model_a is not model_b
    assert data_a is not data_b

    data_a.qpos[0] = 0.5

    assert data_b.qpos[0] == pytest.approx(0.0)


def test_loading_does_not_step_physics(
    tmp_path: Path,
) -> None:
    path = write_model(tmp_path, VALID_MODEL)

    _, data = load_model(path)

    assert data.time == pytest.approx(0.0)


@pytest.mark.parametrize(
    "content",
    [
        "<not_mujoco/>",
        "<mujoco><worldbody>",
        "<mujoco><worldbody/><extra/></mujoco>",
        "\x00",
    ],
)
def test_invalid_content_fails_cleanly(
    tmp_path: Path, content: str
) -> None:
    path = write_model(tmp_path, content)

    with pytest.raises(ModelLoadError):
        load_model(path)
