# ur5e_pose_commander

Type a goal **position and orientation** for the UR5e end effector into the terminal. The robot plans and moves there using
the official [Universal Robots ROS 2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) and MoveIt 2
(ROS 2 Jazzy).

```
 you (terminal)                                                  ros2_control
      │  x y z roll pitch yaw                                  ┌───────────────────────────┐
      ▼                                                         │ scaled_joint_trajectory_  │
 pose_commander ──/move_action──► move_group ──FollowJoint──►   │ controller                │──► mock hardware (Stage 1)
 (this package)   MoveGroup.action  (plan +    Trajectory       │                           │──► Isaac Sim     (Stage 2)
      ▲                              execute)                   │ joint_state_broadcaster   │──► real UR5e     (robot_ip)
      └──────────── TF (base_link → tool0): pose actually reached ◄────────────────────────┘
```

## 1. Install

```bash
sudo apt update
sudo apt install -y ros-jazzy-ur          # ur_description and the rest of the UR stack's dependencies

cd ~/ros2_mowito_test
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y   # dependencies of the packages in src/
colcon build --symlink-install
```

The workspace builds this package together with the UR driver packages, the Robotiq 2F-85 description and driver
(`ros2_robotiq_gripper`), `serial` and `topic_based_ros2_control` (needed for Stage 2), all from `src/`.

Build with the system Python of ROS. If a conda/miniforge environment is active (`(base)` in the prompt), CMake
picks its Python and the build fails with `No module named 'catkin_pkg'`. Run `conda deactivate` first, or turn the
automatic activation off for good with `conda config --set auto_activate_base false`. Then clean the workspace
(`rm -rf build install log`), because the conda Python stays cached in `build/`.

Always build the same way. Switching between `colcon build` and `colcon build --symlink-install` on an existing
workspace fails with `failed to create symbolic link ... Is a directory`. Clean `build/` and `install/` before switching.

## 2. Stage 1: UR5e with fake hardware in RViz

**Terminal 1: driver (mock hardware), MoveIt and RViz**

```bash
source ~/ros2_mowito_test/install/setup.bash
ros2 launch ur5e_pose_commander ur5e_mock.launch.py
```

Wait for `You can start planning now!`. RViz opens with the UR5e and a Robotiq 2F-85 gripper on
`tool0` (planning groups `ur_manipulator` and `gripper`). Add `use_gripper:=false` for the bare arm.

MoveIt runs with the kinematics, planners and controllers of `ur_moveit_config`, but with this package's SRDF
([srdf/ur5e.srdf.xacro](srdf/ur5e.srdf.xacro)). It extends the stock UR SRDF with the `gripper` group (`open` /
`closed`) and the collision pairs MoveIt must skip: the gripper links touch each other by design and sit right on
the wrist. With the stock SRDF and the gripper mounted, MoveIt sees the robot in self-collision in every state, and
every goal fails with `START_STATE_IN_COLLISION`.

This one launch file already starts MoveIt. **Do not also run `ur_moveit.launch.py`**: a second
`move_group` makes goals fail with `CONTROL_FAILED` and `There may be more than one action server
for the action 'move_action'`.

Instead of `ur5e_mock.launch.py` (not in addition to it), the arm-only setup can be started from the
two official launch files by hand (arm only: the stock SRDF of `ur_moveit_config` has no gripper):

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=0.0.0.0 use_mock_hardware:=true launch_rviz:=false
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

**Terminal 2: the pose commander**

```bash
source ~/ros2_mowito_test/install/setup.bash
ros2 run ur5e_pose_commander pose_commander
```

Then type goals at the prompt:

```
UR5e pose commander | group 'ur_manipulator', end effector 'tool0', frame 'base_link'
Current: position [...] m | rpy [...] deg | quat [...]

goal> 0.4 0.2 0.4 180 0 0
Goal   : position [0.400, 0.200, 0.400] m | rpy [180.0, 0.0, 0.0] deg | quat [1.000, 0.000, 0.000, 0.000]
IK: shoulder_pan 9.2, shoulder_lift -97.6, elbow 110.0, wrist_1 -102.4, wrist_2 -90.0, wrist_3 -80.8 deg
Goal accepted, planning and executing...
Goal reached.
Reached: position [0.400, 0.200, 0.401] m | rpy [-180.0, -0.1, -0.0] deg | quat [1.000, -0.000, 0.001, -0.000]

goal> 0.3 -0.3 0.5 90 0 0
...
goal> q
```

### Input format

| Input | Meaning |
|---|---|
| `x y z` | Position in **metres**, keep the current orientation |
| `x y z roll pitch yaw` | Position in metres, orientation in **degrees** (fixed-axis roll/pitch/yaw about X, Y, Z) |
| `x y z qx qy qz qw` | Position in metres, orientation as a quaternion (normalised automatically) |
| `current` | Print the current end-effector pose |
| `help` | Show the format |
| `q` | Quit |

The goal is the pose of `tool0` (the tool flange) in `base_link`. `0.4 0.2 0.4 180 0 0` puts the flange 40 cm in front,
20 cm to the side and 40 cm up, with the tool pointing straight down.

### Options

```bash
ros2 run ur5e_pose_commander pose_commander --goal 0.4 0.2 0.4 180 0 0   # one goal, no prompt
ros2 run ur5e_pose_commander pose_commander --velocity 0.5 --acceleration 0.5
ros2 run ur5e_pose_commander pose_commander --plan-only                  # plan, don't move
```

| Option | Default | Description |
|---|---|---|
| `--goal V...` | – | Goal as 3, 6 or 7 numbers. Runs once without the prompt |
| `--group` | auto | MoveIt planning group. By default the first of `ur_manipulator`, `arm`, `manipulator` that move_group knows (`ur_manipulator` in both stages) |
| `--ee-link` | `tool0` | Link placed at the goal |
| `--frame` | `base_link` | Frame the goal is expressed in |
| `--velocity` / `--acceleration` | `0.2` | Scaling of the joint velocity/acceleration limits (0–1) |
| `--planning-time` | `5.0` | Seconds allowed for planning |
| `--pos-tol` | `0.001` | Position tolerance radius [m], used only when IK fails and the pose is planned directly |
| `--rot-tol` | `0.01` | Orientation tolerance per axis [rad] (~0.57°), same |
| `--plan-only` | off | Only plan, do not execute |
| `--no-floor` | off | Do not add the floor to the planning scene |

## 3. Stage 2: UR5e simulated in Isaac Sim

Same pose commander and the same MoveIt setup, but the arm is now simulated with physics in Isaac Sim instead
of mock hardware.

```
 pose_commander ─► move_group ─► joint_trajectory_controller ─► topic_based_ros2_control
                                                                   │ /isaac_joint_commands ▲ /isaac_joint_states
                                                                   ▼                       │ /clock
                                                          Isaac Sim (UR5e articulation, PhysX joint drives)
```

What changes compared with Stage 1:

| | Stage 1 | Stage 2 |
|---|---|---|
| Robot model | `ur_description` UR5e | the same ([urdf/ur5e_isaac.urdf.xacro](urdf/ur5e_isaac.urdf.xacro)) |
| ros2_control hardware | `mock_components/GenericSystem` | `topic_based_ros2_control/TopicBasedSystem` |
| Trajectory controller | `scaled_joint_trajectory_controller` | `joint_trajectory_controller` (Isaac has no UR speed scaling) |
| MoveIt | `ur_moveit_config` with [srdf/ur5e.srdf.xacro](srdf/ur5e.srdf.xacro) | the same, with [its own controller list](config/ur5e_isaac_moveit_controllers.yaml) |
| Clock | wall time | simulation time from Isaac (`/clock`, `use_sim_time`) |
| Pose commander | `pose_commander` | unchanged |

**Terminal 1: Isaac Sim** (Isaac Sim 6.1 standalone, first start takes a few minutes)

```bash
source /opt/ros/jazzy/setup.bash
~/isaac-sim-standalone-6.1.0-linux-x86_64/python.sh \
    ~/ros2_mowito_test/src/ur5e_pose_commander/isaac/ur5e_isaac_scene.py
```

Wait for `UR5e Isaac Sim scene running`. The script ([isaac/ur5e_isaac_scene.py](isaac/ur5e_isaac_scene.py)) loads a
ground grid and NVIDIA's UR5e asset with its Robotiq 2F-85 gripper variant (`--no-gripper` for the bare arm, then
start the launch file below with `use_gripper:=false`), puts the arm in the same start pose as Stage 1, adds a ROS 2 bridge action graph
and presses Play. Check from another terminal: `ros2 topic hz /isaac_joint_states`.

**Terminal 2: ros2_control, MoveIt and RViz**

```bash
source ~/ros2_mowito_test/install/setup.bash
ros2 launch ur5e_pose_commander ur5e_isaac.launch.py
```

**Terminal 3: the pose commander** (exactly as in Stage 1)

```bash
source ~/ros2_mowito_test/install/setup.bash
ros2 run ur5e_pose_commander pose_commander
goal> 0.4 0.2 0.4 180 0 0
```

The arm moves in Isaac Sim, and RViz shows the same motion (RViz draws the joint states that come back from Isaac).

Notes for Stage 2:

- **Gripper:** plan for the `gripper` group (`open` / `closed`) as in Stage 1. Isaac Sim calls the driven joint
  `finger_joint`, ROS calls it `robotiq_85_left_knuckle_joint`; the scene script translates the name in both
  directions (`ROS_TO_ISAAC_JOINTS`), and also corrects the asset's gripper mount joint, which otherwise leaves the
  gripper hanging ~0.8 m from the wrist under physics.
- **Start order:** Isaac Sim first, then the launch file. When `ur5e_isaac.launch.py` starts, ros2_control commands
  the start pose (from `ur_description/config/initial_positions.yaml`), so if you restart only the ROS side, the arm
  first moves back to the start pose.
- **Real time:** the scene caps Isaac Sim at 60 frames/s with one 1/60 s physics step per frame, so simulation time
  runs at the speed of the wall clock that ros2_control uses for its 100 Hz loop.
- **Tracking is real here:** unlike mock hardware, the simulated joints lag their trajectory slightly. The controller
  waits up to 2 s after the trajectory ends for every joint to settle within 0.01 rad
  ([config/ur5e_isaac_controllers.yaml](config/ur5e_isaac_controllers.yaml)), and aborts if a joint is more than 0.2 rad
  off during the motion.
- **After editing** files in `urdf/`, `srdf/`, `config/`, `launch/` or `isaac/`, rebuild the package
  (`colcon build --symlink-install --packages-select ur5e_pose_commander`): they are installed as copies.

## 4. How it works

0. **Floor.** At startup the script adds a 2 m × 2 m box just below `base_link` to MoveIt's planning scene
   (`/apply_planning_scene`). MoveIt only knows the robot itself; without the floor it plans through the mounting
   surface, and in Isaac Sim the arm then hits the ground.
1. **Input.** The prompt line is parsed into a position and a quaternion (RPY in degrees is converted to a quaternion;
   with only `x y z`, the current orientation is read from TF and kept).
   Bad input is rejected. Goals farther than about 0.85 m (the UR5e reach) trigger a warning.
2. **Inverse kinematics.** The script calls move_group's `/compute_ik` service (KDL solver) to turn the pose into
   six joint angles, with collision checking on. Every UR joint can turn ±360°, so a pose has many IK solutions, and
   some of them can only be reached by swinging the arm through itself. The script therefore seeds IK from the current
   joints and from four elbow-up, tool-down "ready" poses, and keeps the solution nearest to the current joints (the
   one that moves the arm least). It prints the chosen joint angles.
3. **Goal.** The script builds a `moveit_msgs/action/MoveGroup` goal with a *joint constraint* per joint (the IK
   solution, ±0.001 rad). If IK finds no solution, it falls back to *pose constraints* on `tool0`: a sphere of radius
   `--pos-tol` around the target point and the target orientation within `--rot-tol` per axis.
   `start_state.is_diff = true` tells MoveIt to start from the robot's current state.
4. **Planning (move_group).** OMPL (RRTConnect) finds a collision-free joint-space path to the goal. The path is
   time-parameterised against the joint velocity and acceleration limits, scaled by `--velocity` and `--acceleration`.
   If planning fails, the script retries up to 3 times in total, since OMPL is randomised.

   *Why IK first:* with a pose goal, OMPL samples IK solutions itself and may pick one it cannot connect to the start
   in the time allowed. With the default pose goal, about 1 in 3 reachable goals failed. With the nearest IK
   solution as a joint goal, all 10 test goals succeeded on the first attempt.
5. **Execution.** move_group sends the trajectory to the driver's `scaled_joint_trajectory_controller` (a
   `FollowJointTrajectory` action). With mock hardware the commanded positions are copied straight to the joint states.
   On the real robot the driver streams them to the UR controller.
6. **Feedback.** The script waits for the result. It then reads `base_link → tool0` from TF and prints the pose
   actually reached, so you can compare it with the goal.

The script only talks to the standard `/compute_ik` and `/move_action` interfaces. The same script works for mock hardware, Isaac Sim and
the real robot. Only what's running behind move_group changes.

## 5. Limitations: when it breaks

- **Unreachable goals.** Goals beyond about 0.85 m, too close to the base, or with an orientation the wrist can't reach
  at that point make planning fail (reported as `FAILURE`). The 0.85 m check is only a warning. Real
  reachability depends on orientation too.
- **The path is not a straight line.** OMPL plans in joint space. The tool can swing along a curved path to the goal,
  and on a real cell that path can be surprising.
- **Only the robot and the floor are modelled.** The planning scene has the robot's own links and a flat floor at the
  base height. A wall, a fixture or a person isn't in it, so the planner will go through them.
- **Random planner.** RRTConnect is sampling-based. The same goal can give different paths. The final joint
  configuration is fixed by the IK step, but "nearest to the current joints" is only as good as the seeds: KDL is a
  numerical solver and can miss a solution, in which case the script falls back to a pose goal.
- **Retries cost time.** An unreachable goal is planned 3 times before the script gives up (about 15 s).
- **Singularities and joint limits.** Goals near a wrist singularity (wrist 2 ≈ 0) or near ±360° joint limits can fail,
  or produce large joint motions for a small change in pose.
- **Tolerances.** The joint goal is met within 0.001 rad per joint, which is about 1 mm and 0.1° at the tool. In the
  pose fallback the goal is "reached" within `--pos-tol` / `--rot-tol` (1 mm / 0.57°).
- **Roll/pitch/yaw.** RPY has a singularity at pitch = ±90°, so different RPY triples can describe the same orientation.
  The printed RPY of the reached pose can look different from what you typed while the orientation is the same. The
  quaternion is the unambiguous reference.
- **Blocking.** The prompt blocks while the robot moves. There's no way to pre-empt a motion from the prompt, other
  than Ctrl-C, which also stops the script.
- **TF frame.** The goal is for `tool0` (the flange), not the tip of a gripper. Use `--ee-link` if a tool link is in
  the URDF.

## 6. Running on the real UR5e

The script stays the same. Only the driver launch changes:

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=<ROBOT_IP> launch_rviz:=false
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
ros2 run ur5e_pose_commander pose_commander
```

This is the bare arm: the stock driver description and the stock SRDF have no gripper. For the real Robotiq on the
tool connector, pass `description_file:=$(ros2 pkg prefix ur5e_pose_commander)/share/ur5e_pose_commander/urdf/ur5e_robotiq.urdf.xacro use_tool_communication:=true`
to the driver. MoveIt then needs this package's [srdf/ur5e.srdf.xacro](srdf/ur5e.srdf.xacro), started the way
[launch/ur5e_mock.launch.py](launch/ur5e_mock.launch.py) starts move_group. `ur_moveit.launch.py` only loads the stock
SRDF and would reject every goal with `START_STATE_IN_COLLISION`.

What changes:

- **External Control URCap.** The teach pendant must run the *External Control* program pointing at this PC, and the
  robot must be in remote control mode. Otherwise the controller accepts trajectories but the arm doesn't move.
- **Calibration.** Every UR robot has factory kinematic calibration that differs slightly from the nominal URDF. Without
  extracting it (`ur_calibration`) and passing it as `kinematics_params_file`, poses are off by a few millimetres.
- **Real dynamics.** Mock hardware follows the trajectory perfectly. The real arm has tracking error and payload
  effects, and it stops on violations: protective stop on collision, joint limits or safety planes. A goal that works in
  RViz can trigger a protective stop on the real robot.
- **Speed scaling.** The teach pendant's speed slider and the safety system scale the trajectory in real time
  (`scaled_joint_trajectory_controller`). Motion can take longer than planned, or pause.
- **The environment isn't modelled.** Add the table, walls and tools to the planning scene before running, or the
  planner will drive the arm into them. Start with low `--velocity` / `--acceleration` (for example 0.1), keep a hand
  on the e-stop, and preview with `--plan-only` first.
- **Network/real-time.** The driver needs a stable, low-latency Ethernet link. Dropped packets abort the trajectory.
  A real-time kernel is recommended.
- **Payload/TCP.** Set the actual payload and TCP on the robot so that the controller and the safety limits match the
  tool being carried.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `move_group is not running` | Start `ur5e_mock.launch.py` first and wait for `You can start planning now!`. Check that both terminals have the same `ROS_DOMAIN_ID` |
| `Failed: CONTROL_FAILED` with `There may be more than one action server for the action 'move_action'` | Two MoveIt instances are running, usually `ur5e_mock.launch.py` plus `ur_moveit.launch.py`. Stop everything and start only `ur5e_mock.launch.py`. `ros2 node list \| grep -c '^/move_group$'` must print `1` |
| `Failed: START_STATE_IN_COLLISION` (and `IK found no collision-free solution`) | MoveIt runs with a gripper in the URDF but without [srdf/ur5e.srdf.xacro](srdf/ur5e.srdf.xacro), e.g. `ur_moveit.launch.py` with the Robotiq description. Use `ur5e_mock.launch.py` / `ur5e_isaac.launch.py`, or the bare arm |
| Build fails: `No module named 'catkin_pkg'` | CMake used conda's Python. `conda deactivate`, `rm -rf build install log`, rebuild (see [Install](#1-install)) |
| Build fails: `failed to create symbolic link ... Is a directory` | The workspace was built without `--symlink-install` before. `rm -rf build install log`, rebuild |
| `Planning group '...' does not exist` | Wrong `--group` for the running MoveIt config. Leave it out and let the script pick |
| `Failed: FAILURE` | Goal unreachable: move it closer, or change the orientation (for example `180 0 0` = pointing down) |
| `Failed: CONTROL_FAILED` | The controller rejected or aborted the trajectory. Check `ros2 control list_controllers` (`scaled_joint_trajectory_controller` must be `active`) |
| `Cannot read current pose` | TF isn't available yet. Wait a moment after starting the driver |
