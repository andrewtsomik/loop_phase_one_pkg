# loop_phase_one_pkg — UMD Loop Phase I, Challenge S1

A cube in Gazebo Harmonic drives itself to random waypoints while avoiding obstacles with a 2D lidar.

## Requirements
Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (`ros-jazzy-ros-gz`)

## Build
```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone <this repo URL>
cd ~/ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 launch loop_phase_one_pkg sim.launch.py seed:=42
```
`seed:=0` gives new random waypoints each run.

## Files
- `launch/sim.launch.py`: starts Gazebo, spawns the cube, bridges topics, runs the navigator
- `worlds/course.sdf`: obstacle course
- `models/red_cube/`: vehicle with lidar, velocity control, odometry
- `scripts/navigator.py`: waypoint generation and obstacle avoidance

## Troubleshooting

**Cube drives erratically, or the wrong world loads:** old simulation processes are probably still running (for example, after stopping with Ctrl+Z instead of Ctrl+C). Kill them and relaunch:
```bash
pkill -9 -f "gz sim"; pkill -9 -f parameter_bridge; pkill -9 -f navigator.py
ros2 launch loop_phase_one_pkg sim.launch.py seed:=42
```
Always stop the simulation with **Ctrl+C**.

**`executable 'navigator.py' not found`:** the script lost its executable permission.
```bash
chmod +x scripts/navigator.py
```

**Lidar shows no rays / `/scan` is empty (common in VMs):**
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```
then relaunch.

**`RTPS_TRANSPORT_SHM Error` in the output:** harmless. To clear it:
```bash
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```
