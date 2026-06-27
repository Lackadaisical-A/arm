# SO-101 Arm Setup

Local SO-101 / SO-ARM101 LeRobot helpers, ROS2 RViz display/control package, and calibration/config files.

## Windows LeRobot Bridge

Run from a LeRobot checkout with its virtual environment active:

```powershell
cd $env:USERPROFILE\Documents\lerobot
.\activate_lerobot.ps1
python .\scripts_local\rviz_udp_lerobot_bridge.py --port COM6
```

The bridge starts paused. Press `e` in the PowerShell window to enable streaming.

## WSL ROS2 RViz

Run in WSL Ubuntu:

```bash
cd ~/ros2_so101_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch so101_description rviz_udp_control.launch.py
```

Use the calibrated slider GUI. The bridge publishes real robot state back to RViz and commands the arm from the sliders.

## WSL MoveIt RViz Planning

MoveIt 2 is installed in WSL with the `so101_moveit_config` package. For visual planning demos:

```bash
cd ~/ros2_so101_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch so101_moveit_config moveit_demo.launch.py
```

For optional planned-trajectory streaming to the real arm, start the Windows bridge with URDF targets enabled:

```powershell
cd $env:USERPROFILE\Documents\lerobot
.\activate_lerobot.ps1
python .\scripts_local\rviz_udp_lerobot_bridge.py --port COM6 --accept-urdf-targets --max-rate 25 --command-deadband 0.5
```

Then launch MoveIt with real-state feedback and UDP streaming enabled for `Execute`:

```bash
cd ~/ros2_so101_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch so101_moveit_config moveit_real_arm.launch.py
```

Speed up or slow down execution with `time_scale`. For example, this executes planned paths about twice as fast:

```bash
ros2 launch so101_moveit_config moveit_real_arm.launch.py time_scale:=2.0
```

The bridge still starts paused. Press `e` in PowerShell before allowing planned motion to move the real arm. `Plan` previews/animates in RViz; `Execute` sends the trajectory to the SO-101 trajectory controller, which can stream the executed path to the Windows bridge.

## Current Elbow Settings

The working elbow setup is:

- `elbow_flex` servo id: `3`
- Feetech `Phase`: `12`
- `Torque_Limit`: `1000`
- `Minimum_Startup_Force`: `16`
- `P_Coefficient`: `16`

These are applied by `scripts_local/so101_phase_utils.py` for the RViz bridge, keyboard teleop, and replay scripts.
