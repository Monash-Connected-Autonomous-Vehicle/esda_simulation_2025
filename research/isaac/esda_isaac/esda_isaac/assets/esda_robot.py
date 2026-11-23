"""Isaac Lab articulation config for the ESDA IGVC vehicle.

The USD it points at is produced by research/isaac/scripts/convert_robot.py
from research/isaac/assets/esda_robot.urdf, which tools/export_urdf_for_isaac.py
generates from description/robot.urdf.xacro. Nothing in that chain is edited by
hand; regenerate instead.

WHAT THE ARTICULATION LOOKS LIKE AFTER CONVERSION
    merge_fixed_joints folds the chassis, caster, lidar pole, laser_frame,
    camera and GPS into base_link, leaving exactly three rigid bodies and two
    revolute joints:

        /robot/Geometry/base_link               48.400 kg   (articulation root)
        /robot/Geometry/base_link/left_wheel     4.000 kg
        /robot/Geometry/base_link/right_wheel    4.000 kg
                                                56.400 kg

    That total is the budget in research/isaac/config/robot_mass.yaml, not the
    1.25 kg the xacro declares -- see that file for why the xacro's numbers are
    harmless in Gazebo and fatal in PhysX.

VELOCITY CONTROL, NOT POSITION
    stiffness 0.0 with non-zero damping makes an ImplicitActuator a pure
    velocity servo: applied torque is damping x (target - actual) joint
    velocity. That is what diff_drive_controller does on the real robot, and
    it is what the policy's (linear.x, angular.z) action maps onto through
    obs_core.twist_to_wheels. A non-zero stiffness would add a position loop
    that quietly fights every command.

    The USD carries a velocity drive with stiffness 17.5 baked in by the
    importer -- note it ignores the value convert_robot.py requests. It does
    not matter: the actuator config below is applied at spawn time and
    overrides it.
"""
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

_HERE = os.path.dirname(os.path.abspath(__file__))
_ISAAC_DIR = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))

ESDA_ROBOT_USD = os.path.join(_ISAAC_DIR, "usd", "esda_robot", "esda_robot.usda")

# description/robot_core_ref.xacro, cross-checked by tools/validate_isaac_obs.py
# against config/my_controllers_2wd.yaml.
WHEEL_RADIUS = 0.1625
WHEEL_SEPARATION = 0.5

# base_link rides the wheel axle, so the resting height of the root IS the
# wheel radius. The clearance is not cosmetic: spawned flush, the colliders
# start marginally interpenetrating the ground plane and PhysX resolves it by
# applying max_depenetration_velocity continuously -- measured as the robot
# climbing to z = 1.18 m within one second before falling back. Anything
# sampled during that is meaningless, so spawn clear and settle first.
SPAWN_HEIGHT = WHEEL_RADIUS + 0.05

# Seconds of zero-command simulation to run after a reset before trusting any
# measurement. The drop above takes about 0.2 s; this leaves margin.
SETTLE_TIME = 0.75

# Sign relating obs_core's wheel convention to the imported joint axes.
#
# This is +1: the URDF convention survives the import intact, and positive
# wheel velocity drives the robot forward exactly as diff_drive_controller
# intends.
#
# It was briefly -1. Commanding both joints +3.0 rad/s did move the body
# backwards at the correct 0.46 m/s, and a uniform flip looked like the
# obvious explanation. It was not: the robot was spawned upside down by the
# XYZW/WXYZ mix-up in init_state.rot above, so "backwards" was simply the
# other way round. The lesson is that a sign flip is a suspiciously cheap fix
# for a kinematics problem -- it should not have been applied before the
# attitude was verified.
WHEEL_COMMAND_SIGN = 1.0

# The ros2_control command interface in description/ros2_control_2wd.xacro caps
# wheel velocity at +/-10 rad/s. At 0.1625 m that is 1.625 m/s, which is below
# my_controllers_2wd.yaml's 2.0 m/s -- the policy stays at 1.0 m/s, inside both.
WHEEL_VELOCITY_LIMIT = 10.0

# Torque budget, from the vehicle this actually is rather than a guess:
#   accelerate 56.4 kg at the controller's 2.0 m/s^2  ->  112 N  ->  9.1 Nm/wheel
#   hold station on the 8.5 degree ramp in model.sdf  ->   82 N  ->  6.6 Nm/wheel
# 40 Nm leaves roughly 4x headroom for both, and is in the right range for the
# hub motors this class of vehicle uses.
WHEEL_EFFORT_LIMIT = 40.0

# Velocity-servo gain. At 40 Nm the actuator saturates around a 0.8 rad/s error,
# so it tracks commands tightly without becoming a stiff numerical problem.
WHEEL_DAMPING = 50.0


def _stage():
    import omni.usd
    return omni.usd.get_context().get_stage()


def apply_friction_materials(prim_path="/World/Robot", material_root="/World/PhysicsMaterials"):
    """Bind the robot's friction materials AFTER spawning.

    convert_robot.py also writes UsdPhysics materials into the asset, and PhysX
    does not honour them -- converting with and without that pass produced
    byte-identical behaviour. So the authoritative binding happens here, at
    spawn time, through Isaac Lab's own bind_physics_material.

    This is not a detail. With everything falling back to the default 0.5
    friction, the caster -- a sphere 0.725 m behind the axle that
    robot_core_ref.xacro deliberately made nearly frictionless -- resists yaw
    with roughly 56 Nm, while the two drive wheels can only generate about
    50 Nm against the same coefficient. The robot then drives perfectly
    straight and refuses to turn, which is exactly what the square test showed:
    18 m travelled, 0.21 rad of yaw against 6.28 commanded.

    Returns {material name: [bound collider paths]} so callers can check that
    the caster was actually found.
    """
    import yaml
    from pxr import UsdGeom, UsdPhysics

    cfg_path = os.path.join(_ISAAC_DIR, "config", "robot_mass.yaml")
    with open(cfg_path) as fh:
        specs = (yaml.safe_load(fh) or {}).get("materials") or {}

    made = {}
    for name, spec in sorted(specs.items()):
        mat_cfg = sim_utils.RigidBodyMaterialCfg(
            static_friction=float(spec.get("static_friction", 0.5)),
            dynamic_friction=float(spec.get("dynamic_friction", 0.5)),
            restitution=float(spec.get("restitution", 0.0)),
        )
        path = "%s/%s" % (material_root, name)
        mat_cfg.func(path, mat_cfg)

        # RigidBodyMaterialCfg exposes only the two coefficients and
        # restitution, so the combine mode has to be set on the PhysX schema
        # directly. Without it PhysX averages the pair and the caster's 0.02
        # becomes 0.51 against a 1.0 ground -- see robot_mass.yaml.
        mode = str(spec.get("friction_combine_mode", "average"))
        mat_prim = _stage().GetPrimAtPath(path)
        if mat_prim and mat_prim.IsValid():
            from pxr import PhysxSchema
            px = PhysxSchema.PhysxMaterialAPI.Apply(mat_prim)
            px.CreateFrictionCombineModeAttr().Set(mode)
        made[name] = path

    root = _stage().GetPrimAtPath(prim_path)
    bound = {}
    for prim in Usd_iter(root):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        path = str(prim.GetPath())
        low = path.lower()
        # Name first: the caster is now a real swivel-and-roll assembly on
        # continuous joints, so it survives merge_fixed_joints as its own named
        # body. The Sphere fallback is for the legacy asset, where the caster
        # was one rigid sphere merged into base_link and renamed "sphere_N".
        if "caster" in low:
            kind = "caster"
        elif "wheel" in low:
            kind = "wheel"
        elif prim.IsA(UsdGeom.Sphere):
            kind = "caster"
        else:
            kind = "body"
        if kind in made:
            sim_utils.bind_physics_material(path, made[kind])
            bound.setdefault(kind, []).append(path)
    return bound


def Usd_iter(prim):
    """Depth-first walk of a prim and its descendants."""
    if not prim or not prim.IsValid():
        return
    yield prim
    for child in prim.GetChildren():
        for sub in Usd_iter(child):
            yield sub


ESDA_ROBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=ESDA_ROBOT_USD,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            # The vehicle tops out at 1.0 m/s and 0.8 rad/s. These caps exist
            # to stop a bad contact launching a body and poisoning the episode,
            # not to shape normal motion.
            max_linear_velocity=10.0,
            max_angular_velocity=50.0,
            # Leave this at the usual 1.0. Lowering it to 0.1 to soften the
            # spawn transient made things strictly worse, and the measurement
            # is worth recording because the result is counter-intuitive:
            #
            #   1.0 -> robot is shoved up to z = 1.18 m, falls back, settles at
            #          z = 0.1625 and drives correctly thereafter
            #   0.1 -> robot climbs at a CONSTANT 0.0967 m/s, with no
            #          deceleration, and never settles at all
            #
            # A constant-velocity climb means contact is being reported every
            # step and the separation clamp is overriding gravity, so the low
            # value never breaks the contact it is trying to resolve. The high
            # value escapes in one shove. Narrowing the chassis collision box
            # to clear the wheels (robot_mass.yaml) did not change either case,
            # so the contact pair is still unidentified -- a ContactSensor
            # would name it. Until then: spawn clear of the ground and settle
            # on a velocity condition rather than a timer.
            max_depenetration_velocity=1.0,
        ),
        # The converted colliders carry no PhysxCollisionAPI at all, so without
        # this they inherit whatever the scene defaults to. Measured symptom:
        # the robot resting 0.1375 m above the floor with its wheels spinning
        # in the air, and a spawn gap smaller than that counting as
        # penetration -- which is what produced the violent upward shove at
        # reset. rest_offset 0.0 means "touching means touching".
        collision_props=sim_utils.CollisionPropertiesCfg(
            collision_enabled=True,
            contact_offset=0.02,
            rest_offset=0.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, SPAWN_HEIGHT),
        # (x, y, z, w) -- NOT (w, x, y, z). Isaac Lab's InitialStateCfg.rot and
        # ArticulationData.root_quat_w are both XYZW (see QUAT_XYZW_ELEMENT_NAMES
        # on base_articulation_data.root_quat_w), while much of the wider
        # robotics world, this repo's own URDF tooling included, writes WXYZ.
        #
        # Getting it backwards here cost a long debugging session: (1,0,0,0)
        # read as WXYZ looks like the identity, but as XYZW it is a 180 degree
        # roll. The robot spawned upside down with its 1 m lidar pole buried in
        # the ground, PhysX shoved it 1.17 m into the air to resolve the
        # penetration, and it fell back to rest propped on its wheels and the
        # pole tip. Every downstream symptom -- the climb, the tumble, the
        # 69 degree resting pitch -- came from this one tuple.
        rot=(0.0, 0.0, 0.0, 1.0),
        joint_pos={".*_wheel_joint": 0.0},
        joint_vel={".*_wheel_joint": 0.0},
    ),
    actuators={
        # The caster is PASSIVE and must stay that way. The URDF importer gives
        # every joint a velocity drive -- the converted asset reports
        # "[VELOCITY] stiffness 17.5" on caster_steer and caster_wheel_spin
        # just as it does on the drive wheels. A velocity drive holding a
        # target of zero is a brake: it would lock the caster straight and
        # reintroduce exactly the yaw resistance that modelling it properly was
        # meant to remove. Joints not covered by an actuator keep whatever the
        # USD says, so they need covering explicitly with zero gains.
        #
        # The small effort limit and damping are not actuation; they are there
        # because an undamped trailing caster can shimmy, which is a real
        # instability and an unhelpful one to hand a learning policy.
        "caster": ImplicitActuatorCfg(
            joint_names_expr=["caster_steer", "caster_wheel_spin"],
            effort_limit_sim=1.0,
            velocity_limit_sim=100.0,
            stiffness=0.0,
            damping=0.01,
        ),
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=[".*_wheel_joint"],
            # The *_sim variants, not the bare ones. Isaac Lab warns that
            # effort_limit is deprecated, and that velocity_limit "was not
            # getting used by implicit actuators" at all -- so the bare form
            # silently did nothing here.
            effort_limit_sim=WHEEL_EFFORT_LIMIT,
            velocity_limit_sim=WHEEL_VELOCITY_LIMIT,
            stiffness=0.0,          # pure velocity servo -- see module docstring
            damping=WHEEL_DAMPING,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""The vehicle as trained. Spawned at the origin; the env overrides init_state."""
