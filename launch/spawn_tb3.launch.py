import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
import numpy as np

def get_bounding_entity(pose, size):
    min_x = pose[0] - size[0] / 2.0
    max_x = pose[0] + size[0] / 2.0
    min_y = pose[1] - size[1] / 2.0
    max_y = pose[1] + size[1] / 2.0
    return (min_x, max_x, min_y, max_y)

# TODO: Fix so objects don't spwan inside eachother
def is_point_inside_entities(x, y, entities):
    for entity in entities:
        min_x, max_x, min_y, max_y = get_bounding_entity(entity['pose'], entity['size'])
        if min_x < x < max_x and min_y < y < max_y:
            return True
    return False

DEFAULT_GOAL = 'fire_hydrant_small'

def launch_setup(context, *args, **kwargs):

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    seed = LaunchConfiguration('myseed')

    num_entities = int(LaunchConfiguration('num_entities').perform(context))

    seed_index = int(seed.perform(context))

    np.random.seed(1337 * seed_index)
    
    this_path = get_package_share_directory('rebet_frog')
    custom_world_launch_path = os.path.join(
        this_path, 'launch', 'custom_world.launch.py')


    entities = []
    robot_position = {'pose':(0.0,0.0,0.0), 'size': (1, 1, 1)}
    
    for _ in range(num_entities):
        entity_x_random = np.random.uniform(low=-1.2, high=1.8)
        entity_y_random = np.random.uniform(low=-1.8, high=1.8)

        while(is_point_inside_entities(entity_x_random,entity_y_random, entities + [robot_position])):
            entity_x_random = np.random.uniform(low=-1.2, high=2)
            entity_y_random = np.random.uniform(low=-1.8, high=1.8)

        entity = {'pose': (entity_x_random, entity_y_random, -0.14), 'size': (3.0, 2, 1.2)}

        entities.append(entity)

    tb_x_pose = LaunchConfiguration('x_pose', default=str(0.0))
    tb_y_pose = LaunchConfiguration('y_pose', default=str(0.0))

    custom_world =  IncludeLaunchDescription(
        AnyLaunchDescriptionSource(custom_world_launch_path),
        launch_arguments={
           'gui': LaunchConfiguration('gui'),
        }.items())

    delayed_spawn_goals = []
    for idx, entity in enumerate(entities):
        spawn_goal = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', f'entity_{idx}',
                '-database', LaunchConfiguration('goal_object'),
                '-x', str(entity['pose'][0]),
                '-y', str(entity['pose'][1]),
                '-z', '-0.14',
            ],
            output='screen',
        )    
        delayed_spawn_goal = TimerAction(
            period=5.0,  # Wait for 5 seconds before spawning TurtleBot
            actions=[spawn_goal]
        )
        delayed_spawn_goals.append(delayed_spawn_goal)

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': tb_x_pose,
            'y_pose': tb_y_pose
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return [custom_world] + delayed_spawn_goals + [spawn_turtlebot_cmd, robot_state_publisher_cmd]

def generate_launch_description():

    using_waffle = SetEnvironmentVariable(name="TURTLEBOT3_MODEL",value="waffle")

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run with gui (true/false)')

    seed_arg = DeclareLaunchArgument(
        'myseed',
        default_value='0',
        description='seed')
    
    num_entities_arg = DeclareLaunchArgument(
        'num_entities',
        default_value='4',
        description='Number of entities to spawn'
    )
    
    goal_arg = DeclareLaunchArgument(
        'goal_object',
        default_value=DEFAULT_GOAL,
        description='Object spawned in for FROG to find'
    )


    return LaunchDescription([
        using_waffle,
        seed_arg,
        num_entities_arg,
        gui_arg,
        goal_arg,
        OpaqueFunction(function=launch_setup),
    ])