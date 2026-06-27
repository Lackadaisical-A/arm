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

## Current Elbow Settings

The working elbow setup is:

- `elbow_flex` servo id: `3`
- Feetech `Phase`: `12`
- `Torque_Limit`: `1000`
- `Minimum_Startup_Force`: `800`

These are applied by `scripts_local/so101_phase_utils.py` for the RViz bridge, keyboard teleop, and replay scripts.
