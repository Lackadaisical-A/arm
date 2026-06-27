# SO-101 RViz Description

This package wraps the SO-101 6DOF URDF from:

https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf

The URDF mesh paths were converted from relative `assets/...` paths to
`package://so101_description/assets/...` so RViz can load the STL files.

## Build

Run these commands inside a ROS2 environment:

```bash
cd ~/Documents/lerobot/ros2_so101_ws
colcon build --symlink-install
source install/setup.bash
```

On Windows PowerShell with a native ROS2 install, use:

```powershell
cd $env:USERPROFILE\Documents\lerobot\ros2_so101_ws
colcon build --symlink-install
.\install\setup.ps1
```

## View With Joint Sliders

```bash
ros2 launch so101_description display.launch.py
```

## Forward RViz Slider Targets To The Windows LeRobot Bridge

Start the Windows bridge first:

```powershell
cd $env:USERPROFILE\Documents\lerobot
.\activate_lerobot.ps1
python .\scripts_local\rviz_udp_lerobot_bridge.py --port COM6
```

Then launch RViz and the UDP publisher in WSL:

```bash
ros2 launch so101_description rviz_udp_control.launch.py
```

The Windows bridge starts paused. Press `e` in the PowerShell bridge window to
enable command streaming after the arm is clear.

## Display The Real Robot State In RViz

Start the WSL/RViz receiver:

```bash
cd ~/ros2_so101_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch so101_description display_real_state.launch.py
```

Start the Windows real-state publisher:

```powershell
cd $env:USERPROFILE\Documents\lerobot
.\activate_lerobot.ps1
python .\scripts_local\publish_lerobot_state_udp.py --port COM6
```

This is display-only. It reads the real servo positions and publishes them to
RViz; it does not command the servos.

## Play A Saved Manual Recording In RViz

```bash
ros2 launch so101_description play_recording.launch.py recording:=/absolute/path/to/recording.json
```

The recording JSON files created by `manual_record_replay_so101.py` are saved in:

```text
C:\Users\herma\Documents\lerobot\recordings
```
