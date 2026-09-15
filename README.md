# loop_phase_one_pkg — UMD Loop Phase I, Challenge S1

A cube in Gazebo Harmonic drives itself to random waypoints while avoiding obstacles with a 2D lidar.

## Video
[Software Challenge One Demonstration](https://www.youtube.com/watch?v=y4UdAa3LrQs)

## Build
```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/andrewtsomik/loop_phase_one_pkg
cd ~/ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 launch loop_phase_one_pkg sim.launch.py
```

## Files
- `launch/sim.launch.py`: starts Gazebo, spawns the cube, bridges topics, runs the navigator
- `worlds/course.sdf`: obstacle course
- `models/red_cube/`: vehicle with lidar, velocity control, odometry
- `scripts/navigator.py`: waypoint generation and obstacle avoidance

## Troubleshooting

**Cube drives erratically, or the wrong world loads:** old simulation processes are probably still running (for example, after stopping with Ctrl+Z instead of Ctrl+C). Kill them and relaunch:
```bash
pkill -9 -f "gz sim"; pkill -9 -f parameter_bridge; pkill -9 -f navigator.py
ros2 launch loop_phase_one_pkg sim.launch.py
```
Always stop the simulation with **Ctrl+C**.

