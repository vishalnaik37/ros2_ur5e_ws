"""Stage 2: UR5e scene in Isaac Sim with a ROS 2 joint-state bridge.

Run it with Isaac Sim's Python (not the system Python):
    ~/Downloads/isaac-sim-standalone-6.1.0-linux-x86_64/python.sh ur5e_isaac_scene.py [--headless]

It builds the scene (ground grid + NVIDIA's UR5e asset, by default with its Robotiq 2F-85 gripper
variant), starts the simulation, and adds an OmniGraph action graph that, on every physics tick,
  - publishes the simulated joint states on /isaac_joint_states (sensor_msgs/JointState)
  - publishes the simulation time on /clock
  - subscribes to /isaac_joint_commands and feeds those position targets to the joint drives
On the ROS side, topic_based_ros2_control (ur5e_isaac.launch.py) uses these two topics as its
hardware, so ros2_control, MoveIt and the pose commander work as with mock hardware.

The gripper's driven joint is called finger_joint in Isaac Sim and robotiq_85_left_knuckle_joint in ROS
(robotiq_description); the main loop translates the names in both directions (ROS_TO_ISAAC_JOINTS).
Its other joints are PhysX mimic joints and follow finger_joint. Start ur5e_isaac.launch.py with the
same gripper setting (use_gripper:=false together with --no-gripper).
"""

import argparse
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
parser.add_argument("--headless", action="store_true", help="run without the Isaac Sim window")
parser.add_argument("--frames", type=int, default=0, help="stop after N frames (0 = run until closed)")
parser.add_argument("--no-gripper", action="store_true", help="UR5e without the Robotiq 2F-85 gripper")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless})

# Isaac Sim modules can only be imported once the app is running.
import carb  # noqa: E402
import carb.settings  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import usdrt.Sdf  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402
from isaacsim.core.rendering_manager import ViewportManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402

ROBOT_PATH = "/World/ur5e"
# The asset puts PhysicsArticulationRootAPI on its fixed base joint, not on the robot prim.
ARTICULATION_ROOT = ROBOT_PATH + "/root_joint"
ROBOT_USD = "/Isaac/Robots_Multiphysics/UniversalRobots/ur5e/ur5e.usda"
ENVIRONMENT_USD = "/Isaac/Environments/Grid/default_environment.usd"
GRAPH_PATH = "/World/ROS2Bridge"

# Isaac Sim joint names that differ from the ROS (URDF) ones. Isaac's finger_joint is the 2F-85's driven
# joint: 0 = open, 0.785 rad = closed, the same range as robotiq_85_left_knuckle_joint.
ROS_TO_ISAAC_JOINTS = {"robotiq_85_left_knuckle_joint": "finger_joint"}
ISAAC_TO_ROS_JOINTS = {v: k for k, v in ROS_TO_ISAAC_JOINTS.items()}

# Start pose, same as the UR driver's mock hardware (ur_description/config/initial_positions.yaml).
INITIAL_JOINTS_DEG = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -90.0,
    "elbow_joint": 0.0,
    "wrist_1_joint": -90.0,
    "wrist_2_joint": 0.0,
    "wrist_3_joint": 0.0,
}

PHYSICS_HZ = 60

# Run in real time: one app frame = one physics step of 1/PHYSICS_HZ s. Without the rate limit a
# standalone app (especially headless) runs as fast as it can, so simulation time runs ahead of the
# 100 Hz wall-clock ros2_control loop and the arm lags behind its trajectory.
settings = carb.settings.get_settings()
settings.set("/app/runLoops/main/rateLimitEnabled", True)
settings.set("/app/runLoops/main/rateLimitFrequency", PHYSICS_HZ)

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

assets_root = get_assets_root_path()
if assets_root is None:
    carb.log_error("Could not find the Isaac Sim assets folder (needs internet or a local Nucleus).")
    simulation_app.close()
    sys.exit(1)

# ---------------------------------------------------------------- scene
stage_utils.set_stage_units(meters_per_unit=1.0)
stage_utils.add_reference_to_stage(assets_root + ENVIRONMENT_USD, "/World/environment")
stage_utils.add_reference_to_stage(assets_root + ROBOT_USD, ROBOT_PATH)
stage = stage_utils.get_current_stage()
if not args.no_gripper:
    # The asset ships the Robotiq 2F-85 as a variant, attached to wrist_3_link and merged into the arm's
    # articulation.
    stage.GetPrimAtPath(ROBOT_PATH).GetVariantSets().GetVariantSet("Gripper").SetVariantSelection("robotiq_2f_85")
    simulation_app.update()
    # The variant's mount joint has the flange's *world* position (0.817, 0.233, 0.063) as its frame on the
    # gripper body, so under physics the gripper hangs ~0.8 m away from the wrist on a rigid lever, hits the
    # ground and drags the arm. Recompute that frame from the authored (zero) pose, where the gripper does sit
    # on the flange.
    mount = UsdPhysics.FixedJoint(stage.GetPrimAtPath(f"{ROBOT_PATH}/joints/robot_gripper_joint"))
    xforms = UsdGeom.XformCache()
    wrist = xforms.GetLocalToWorldTransform(stage.GetPrimAtPath(str(mount.GetBody0Rel().GetTargets()[0])))
    gripper = xforms.GetLocalToWorldTransform(stage.GetPrimAtPath(str(mount.GetBody1Rel().GetTargets()[0])))
    frame0 = Gf.Matrix4d().SetRotate(Gf.Quatd(mount.GetLocalRot0Attr().Get()))
    frame0.SetTranslateOnly(Gf.Vec3d(mount.GetLocalPos0Attr().Get()))
    frame1 = frame0 * wrist * gripper.GetInverse()  # joint frame in the gripper body's coordinates
    mount.GetLocalPos1Attr().Set(Gf.Vec3f(frame1.ExtractTranslation()))
    mount.GetLocalRot1Attr().Set(Gf.Quatf(frame1.ExtractRotationQuat()))
simulation_app.update()

# Put every joint (and its drive target) at the start pose, so the arm does not swing on Play.
for joint, deg in INITIAL_JOINTS_DEG.items():
    prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/joints/{joint}")
    if not prim.IsValid():
        carb.log_error(f"Joint {joint} not found under {ROBOT_PATH}/joints")
        continue
    prim.GetAttribute("drive:angular:physics:targetPosition").Set(deg)
    state = prim.GetAttribute("state:angular:physics:position")
    if state:
        state.Set(deg)

ViewportManager.set_camera_view(
    "/OmniverseKit_Persp", eye=np.array([1.6, 1.4, 1.1]), target=np.array([0.0, 0.0, 0.4]))

# ------------------------------------------------------- ROS 2 bridge graph
og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.CONNECT: [
            # simulated joint states -> /isaac_joint_states
            # (jointNames are not connected here: the main loop sets them, translated, for both directions,
            # and only then connects ReadJointState's execOut to PublishJointState, so it never publishes
            # with an empty name list on the first tick)
            ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
            ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
            ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
            ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
            ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
            ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
            ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
            # /isaac_joint_commands -> joint drive targets
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            # (ArticulationController's execIn is connected by the main loop once it has the command's joint
            # names, so it never applies a command with an empty or stale name list)
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            # simulation time -> /clock
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
            ("Context.outputs:context", "SubscribeJointState.inputs:context"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ArticulationController.inputs:robotPath", ARTICULATION_ROOT),
            ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(ARTICULATION_ROOT)]),
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
        ],
    },
)
simulation_app.update()

# ------------------------------------------------------------- run
SimulationManager.setup_simulation(dt=1.0 / PHYSICS_HZ, device="cpu")
app_utils.play()
simulation_app.update()
print("UR5e Isaac Sim scene running: publishing /isaac_joint_states and /clock, "
      "listening on /isaac_joint_commands.", flush=True)


def translated(attr_in, attr_out, mapping, last):
    """Copy a joint-name list from one graph attribute to another, renaming via mapping, when it changes."""
    names = [str(n) for n in og.Controller.get(og.Controller.attribute(f"{GRAPH_PATH}/{attr_in}")) or []]
    if names != last:
        og.Controller.set(og.Controller.attribute(f"{GRAPH_PATH}/{attr_out}"), [mapping.get(n, n) for n in names])
    return names


state_names, command_names = None, None
publishing = commanding = False
frame = 0
while simulation_app.is_running():
    simulation_app.update()
    # The names only change when the articulation or the ROS command message changes, i.e. once at start.
    state_names = translated(
        "ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames", ISAAC_TO_ROS_JOINTS, state_names)
    if state_names and not publishing:
        og.Controller.connect(
            f"{GRAPH_PATH}/ReadJointState.outputs:execOut", f"{GRAPH_PATH}/PublishJointState.inputs:execIn")
        publishing = True
    command_names = translated(
        "SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames",
        ROS_TO_ISAAC_JOINTS, command_names)
    if command_names and not commanding:
        og.Controller.connect(
            f"{GRAPH_PATH}/OnPlaybackTick.outputs:tick", f"{GRAPH_PATH}/ArticulationController.inputs:execIn")
        commanding = True
    frame += 1
    if args.frames and frame >= args.frames:
        break

app_utils.stop()
simulation_app.close()
