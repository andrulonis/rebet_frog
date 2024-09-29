# rebet_frog
ReBeT applied a to a Turtlebot

0. I am assuming you have ROS2 Humble, Gazebo Classic, and Navigation 2 installed.

1. Create a workspace folder, preferrably in your home folder:
```bash
mkdir -p ~/rebet_ws/src
```
2. Clone the dependencies using VCS:
```bash
cd rebet_ws
vcs import --input https://raw.githubusercontent.com/EGAlberts/rebet_frog/refs/heads/main/frog.rosinstall src
```

3. Source your ROS2 Humble installation and build everything except rebet_samples.
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip rebet_samples
```

