# rebet_frog
ReBeT applied a to a Turtlebot

0. I am assuming you have ROS2 Humble, Gazebo Classic, and Navigation 2 installed.

1. Create a workspace folder, preferrably in your home folder:
```bash
mkdir -p ~/rebet_ws/src
```

2. Clone this repository. For example:
```bash
git clone https://github.com/EGAlberts/rebet_frog.git
```
Keep in mind if you are using a specific branch or commit version that should be reflected in the clone, or remedied after cloning the default branch.

3. Clone the dependencies using VCS:
```bash
cd ~/rebet_ws
vcs import --input src/rebet_frog/frog.rosinstall src
```

3. Source your ROS2 Humble installation, install dependencies using rosdep
```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```
4. To be sure, you should manually install ultralytics
```bash
pip install ultralytics
```
as well as masced_bandits
```bash
pip install masced_bandits
```

5. Build everything
```bash
colcon build --symlink-install
```