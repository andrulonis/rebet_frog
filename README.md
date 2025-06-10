# rebet_frog
ReBeT applied a to a Turtlebot

## Prequisites:
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo Classic](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
- [Nav2](https://docs.nav2.org/getting_started/index.html#installation)

## Installation:

1. Create a workspace folder, preferrably in your home folder:
```bash
mkdir -p ~/rebet_ws/src
```

2. Clone this repository. For example:
```bash
git clone https://github.com/andrulonis/rebet_frog.git
```
Keep in mind if you are using a specific branch or commit version that should be reflected in the clone, or remedied after cloning the default branch.

3. Clone the dependencies using VCS:
```bash
cd ~/rebet_ws
vcs import --input src/rebet_frog/frog.rosinstall src
```

4. Source your ROS2 Humble installation, install dependencies using rosdep
```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```
5. To be sure, you should manually install ultralytics
```bash
pip install ultralytics
```
as well as masced_bandits
```bash
pip install masced_bandits
```

6. Build everything
```bash
colcon build --symlink-install
```

7. Required for using [PRISM](https://www.prismmodelchecker.org) adaptation:
```bash
cd ~/rebet_ws/src/aal/prism-4.8.1-linux64-x86
./install.sh
```

8. Download YOLOv8x model from [here](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8x.pt) and place it in ```~/rebet_ws/src/rebet_frog/config/``` - this limitation comes from the fact that Git LFS cannot be used on forks and the file's size is more than 100MB.

## Usage:
There are myriad launch files which need to be executed. No matter which configuration of rebet_frog you want to use the following launch files should be used:
Make sure you always have the workspace sourced when running these commands (```source ~/rebet_ws/install/setup.bash```).
### Gazebo Classic
Note: you may need to source this first for your gazebo classic to work ```bash source /usr/share/gazebo/setup.bash```.
```bash
ros2 launch rebet_frog spawn_tb3.launch.py gui:=true myseed:=1
```
For launching the simulation and spawning the turtlebot3 waffle into the world.

### Arborist
```bash
ros2 launch rebet_frog arborist_config_launch.py
```
For managing the behaviour trees.

### AAL
```bash
ros2 run aal adaptation_layer
```
For providing architectural adaptations of ROS2 Nodes.

### System Reflection
```bash
ros2 run rebet_frog system_reflection.py
```
For collecting and storing system information in the BT's knowledge.

### Navigation 2
Depending on whether you plan to perform SLAM or use a pre-made map can be specified through the use_map launch argument. When use_map is false, slam_toolbox is launched to create a map.

For SLAM:
```bash
ros2 launch rebet_frog navigation_launch.py use_map:=false
```
and for the pre-made map:
```bash
ros2 launch rebet_frog navigation_launch.py use_map:=true
```

If you are choosing to perform SLAM, you can automate the process with frontier exploration:
```bash
ros2 launch wavefront_frontier frontier_launch.py 
```

### YOLO
Some of the missions expect YOLO to be running for object detection.
```bash
ros2 launch rebet_frog yolo_self_start_launch.py 
```

### Start Mission
Ultimately, to start the mission you need to use the following command:
```bash
ros2 run rebet_frog tree_action_client.py BT_NAME
```
where BT_NAME matches the name (ID) of a behaviour tree defined in the [trees folder](/trees).
Right now, TOAD_PRISM, TOAD_RANDOM, TOAD_GREEDY, TOAD_CONSERVATIVE are provided.

### PRISM adaptation strategies
In the case one of the PRISM adaptation strategies is used:
In the behaviour tree, you must provide absolute path to the directory with the PRISM model and other required files in the AdaptNode of the tree as a parameter ```model_dir="/absolute/path/to/model/dir"```. The directory should contain following files: `base_model.pm`, `properties.pctl`, and `utility_function.py` in case of DTMC/CTMC and `base_model.pm` and `property.pctl` in case of MDP.
The `utility_function.py` file should define a function named calculate_utility(prop_results), which takes as argument a list corresponding to the results of the properties defined in the properties file. It should return a floating point number indicating the utility associated to the results of the properties.

In the case of the Markov Chain adaptation strategy, if any of the configuration paramaters can come in the form of strings, you should provide a file named `string_config_params.txt` that lists all the names of string parameters and their possible values (also works for string arrays). They will be treated as an int value, similar to an enum. This arises from the limitation of PRISM not supporting strings. Each line of the file should look as follows:
```
string_var_name str_value0 str_value1 ...
```