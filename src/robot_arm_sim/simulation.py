from pathlib import Path

import mujoco


ASSET_DIR = Path(__file__).parent / "assets"
# The pedestal stands beside the table's -X edge; the Panda faces world +X.
# Its mounting surface is level with the tabletop, leaving a 0.08 m gap
# between the pedestal's +X face and the table's -X edge.
ROBOT_BASE_POSITION = (-0.65, 0.0, 0.5)
DEFAULT_PANDA_XML = (
    Path.cwd()
    / ".cache"
    / "robot_models"
    / "franka_emika_panda"
    / "panda.xml"
)


def build_scene(
    panda_path: str | Path = DEFAULT_PANDA_XML,
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Compose the packaged task scene with a local Panda model."""
    panda_path = Path(panda_path).expanduser().resolve()
    if not panda_path.is_file():
        raise ValueError(
            f"Panda model not found at {panda_path}. Provide a locally available, "
            "reviewed model with --robot-model; see docs/assets_and_licenses.md "
            "for the model source and asset requirements."
        )

    scene = mujoco.MjSpec.from_file(str(ASSET_DIR / "demo.xml"))
    panda = mujoco.MjSpec.from_file(str(panda_path))
    panda_model = panda.compile()
    if panda_model.nbody < 2 or any(
        panda_model.body_jntnum[b] != 0
        for b in range(1, panda_model.nbody)
        if panda_model.body_parentid[b] == 0
    ):
        raise ValueError("Robot base must be a fixed body with no joints")
    home_id = mujoco.mj_name2id(panda_model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_id < 0:
        raise ValueError("Panda model is missing its 'home' keyframe")
    panda_data = mujoco.MjData(panda_model)
    mujoco.mj_resetDataKeyframe(panda_model, panda_data, home_id)

    mount = scene.worldbody.add_body(name="robot_mount", pos=ROBOT_BASE_POSITION)
    mount.add_geom(
        name="robot_pedestal",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0, 0, -0.25],
        size=[0.12, 0.12, 0.25],
        rgba=[0.3, 0.32, 0.35, 1],
        contype=1,
        conaffinity=1,
    )
    scene.attach(panda, prefix="", frame=mount.add_frame())

    model = scene.compile()
    data = mujoco.MjData(model)

    qpos_widths = {
        int(mujoco.mjtJoint.mjJNT_FREE): 7,
        int(mujoco.mjtJoint.mjJNT_BALL): 4,
        int(mujoco.mjtJoint.mjJNT_SLIDE): 1,
        int(mujoco.mjtJoint.mjJNT_HINGE): 1,
    }
    qvel_widths = {
        int(mujoco.mjtJoint.mjJNT_FREE): 6,
        int(mujoco.mjtJoint.mjJNT_BALL): 3,
        int(mujoco.mjtJoint.mjJNT_SLIDE): 1,
        int(mujoco.mjtJoint.mjJNT_HINGE): 1,
    }
    for panda_joint_id in range(panda_model.njnt):
        joint_name = mujoco.mj_id2name(
            panda_model, mujoco.mjtObj.mjOBJ_JOINT, panda_joint_id
        )
        if joint_name is None:
            continue
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        if joint_id < 0:
            raise ValueError(f"Attached Panda joint is missing from the scene: {joint_name}")
        joint_type = int(panda_model.jnt_type[panda_joint_id])
        panda_qpos = panda_model.jnt_qposadr[panda_joint_id]
        scene_qpos = model.jnt_qposadr[joint_id]
        qpos_width = qpos_widths[joint_type]
        data.qpos[scene_qpos : scene_qpos + qpos_width] = panda_data.qpos[
            panda_qpos : panda_qpos + qpos_width
        ]

        panda_qvel = panda_model.jnt_dofadr[panda_joint_id]
        scene_qvel = model.jnt_dofadr[joint_id]
        qvel_width = qvel_widths[joint_type]
        data.qvel[scene_qvel : scene_qvel + qvel_width] = panda_data.qvel[
            panda_qvel : panda_qvel + qvel_width
        ]

    for panda_actuator_id in range(panda_model.nu):
        actuator_name = mujoco.mj_id2name(
            panda_model, mujoco.mjtObj.mjOBJ_ACTUATOR, panda_actuator_id
        )
        if actuator_name is None:
            continue
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
        )
        if actuator_id < 0:
            raise ValueError(
                f"Attached Panda actuator is missing from the scene: {actuator_name}"
            )
        data.ctrl[actuator_id] = panda_data.ctrl[panda_actuator_id]

    mujoco.mj_forward(model, data)
    validate_robot_placement(model, data)
    validate_ball_spawn(model, data)
    return model, data


def validate_ball_spawn(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Reject overlapping ball reset poses after mj_forward; touching is valid."""
    ball_id = model.body("ball").id
    ball_geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] == ball_id]
    for ball in ball_geoms:
        for other in range(model.ngeom):
            if model.geom_bodyid[other] == ball_id:
                continue
            if not (
                (model.geom_contype[ball] & model.geom_conaffinity[other])
                or (model.geom_contype[other] & model.geom_conaffinity[ball])
            ):
                continue
            distance = mujoco.mj_geomDistance(model, data, ball, other, 0.01, None)
            if distance < -1e-6:
                name = model.geom(other).name or f"geom {other}"
                raise ValueError(
                    f"Ball spawn overlaps {name} by {-distance:.6f} m; "
                    "choose a position on or above the support surface "
                    "and clear of robot geometry"
                )


def validate_robot_placement(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Reject robot penetration into scene supports at the current pose.

    Query collision distances directly: fixed-body pairs may be absent from
    data.contact. A 10 micrometre tolerance allows numerical roundoff, with
    0.1 mm allowed for the fixed base mesh against its mounting surface.
    Touching the mounting surface is intentional. This checks a pose, not all
    possible joint configurations; call it after changing reset positions.
    """
    mount_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot_mount")
    if mount_id < 0:
        return  # The robot-free demo has no mount.
    robot_bodies = {mount_id}
    for body in range(mount_id + 1, model.nbody):
        if int(model.body_parentid[body]) in robot_bodies:
            robot_bodies.add(body)
    robot_bodies.remove(mount_id)
    robot_geoms = [
        g for g in range(model.ngeom) if int(model.geom_bodyid[g]) in robot_bodies
    ]
    table_id = model.body("table").id
    supports = [
        g for g in range(model.ngeom)
        if int(model.geom_bodyid[g]) in (0, table_id, mount_id)
    ]
    for robot in robot_geoms:
        for support in supports:
            if not (
                (model.geom_contype[robot] & model.geom_conaffinity[support])
                or (model.geom_contype[support] & model.geom_conaffinity[robot])
            ):
                continue
            distance = mujoco.mj_geomDistance(model, data, robot, support, 0.01, None)
            fixed_mount_contact = (
                model.geom_bodyid[support] == mount_id
                and model.body_weldid[model.geom_bodyid[robot]]
                == model.body_weldid[mount_id]
            )
            tolerance = 1e-4 if fixed_mount_contact else 1e-5
            if distance < -tolerance:
                robot_name = model.geom(robot).name or f"geom {robot}"
                support_name = model.geom(support).name or f"geom {support}"
                raise ValueError(
                    f"Robot placement overlaps {support_name}: {robot_name} "
                    f"penetrates by {-distance:.6f} m"
                )
