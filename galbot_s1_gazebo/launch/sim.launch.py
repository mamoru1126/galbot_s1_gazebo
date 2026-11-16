"""ROS 2 Jazzy + Gazebo Harmonic: Galbot S1 table demo with web UI.

    ros2 launch galbot_s1_gazebo sim.launch.py               # Gazebo GUI + web UI
    ros2 launch galbot_s1_gazebo sim.launch.py gui:=false    # headless (web UI only)

Web UI: http://<host>:8000   (rosbridge ws://<host>:9090)
"""
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONTROLLERS = ['joint_state_broadcaster', 'left_arm_controller', 'right_arm_controller',
               'left_gripper_controller', 'right_gripper_controller', 'torso_controller',
               'head_controller', 'steering_controller', 'wheel_controller']
CAMERAS = ['overview_camera', 'head_camera', 'right_wrist_camera', 'left_wrist_camera']


def bridge_config(scene, cameras):
    def entry(topic, ros_type, gz_type, direction):
        return {'ros_topic_name': topic, 'gz_topic_name': topic, 'ros_type_name': ros_type,
                'gz_type_name': gz_type, 'direction': direction}
    cfg = [
        entry('/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock', 'GZ_TO_ROS'),
        entry('/galbot/base/cmd_vel', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ'),
        entry('/galbot/ground_truth/odom', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'),
        entry('/galbot/sim/attach', 'std_msgs/msg/String', 'gz.msgs.StringMsg', 'ROS_TO_GZ'),
        entry('/galbot/sim/detach', 'std_msgs/msg/String', 'gz.msgs.StringMsg', 'ROS_TO_GZ'),
        entry('/galbot/sim/set_model_pose', 'std_msgs/msg/String', 'gz.msgs.StringMsg', 'ROS_TO_GZ'),
        entry('/galbot/sim/attached', 'std_msgs/msg/String', 'gz.msgs.StringMsg', 'GZ_TO_ROS'),
    ]
    for obj in scene['objects']:
        cfg.append(entry('/galbot/objects/%s/odom' % obj['name'], 'nav_msgs/msg/Odometry',
                         'gz.msgs.Odometry', 'GZ_TO_ROS'))
    if cameras:
        for cam in CAMERAS:
            cfg.append(entry('/galbot/%s/image_raw' % cam, 'sensor_msgs/msg/Image', 'gz.msgs.Image', 'GZ_TO_ROS'))
            cfg.append(entry('/galbot/%s/camera_info' % cam, 'sensor_msgs/msg/CameraInfo',
                             'gz.msgs.CameraInfo', 'GZ_TO_ROS'))
    return cfg


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    gui = arg('gui').lower() == 'true'
    cameras = arg('cameras').lower() == 'true'
    web = arg('web').lower() == 'true'
    rviz = arg('rviz').lower() == 'true'

    gz_share = get_package_share_directory('galbot_s1_gazebo')
    scene_file = arg('scene_file') or os.path.join(gz_share, 'config', 'scene.yaml')
    with open(scene_file) as f:
        scene = yaml.safe_load(f)
    world_file = arg('world') or os.path.join(gz_share, 'worlds', 'table_scene.sdf')
    world_name = scene.get('world_name', 'galbot_table_world')

    robot_description = xacro.process_file(
        os.path.join(gz_share, 'urdf', 'galbot_s1_sim.urdf.xacro'),
        mappings={'sim': 'gz', 'cameras': 'true' if cameras else 'false',
                  'controllers_file': os.path.join(gz_share, 'config', 'ros2_controllers.yaml')}).toxml()

    bridge_file = os.path.join(tempfile.gettempdir(), 'galbot_s1_bridge.yaml')
    with open(bridge_file, 'w') as f:
        yaml.safe_dump(bridge_config(scene, cameras), f)

    gz_args = '-r -v 3 ' + world_file if gui else '-r -s -v 3 --headless-rendering ' + world_file
    actions = [
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
                                  os.path.join(get_package_prefix('galbot_s1_gazebo'), 'lib')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='galbot_gz_bridge',
             output='screen', parameters=[{'config_file': bridge_file, 'use_sim_time': True}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
    ]
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', world_name, '-topic', 'robot_description',
                            '-name', scene['robot']['name'], '-x', '0', '-y', '0', '-z', '0.002'])
    spawner = Node(package='controller_manager', executable='spawner', output='screen',
                   arguments=CONTROLLERS + ['--controller-manager', '/controller_manager',
                                            '--controller-manager-timeout', '180'])
    actions += [spawn, RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[spawner]))]

    common = {'use_sim_time': True}
    actions += [
        Node(package='galbot_s1_control', executable='base_controller.py', name='galbot_base_controller',
             output='screen', parameters=[common]),
        Node(package='galbot_s1_control', executable='commander.py', name='galbot_commander',
             output='screen', parameters=[common, {'robot_description': robot_description,
                                                   'scene_file': scene_file,
                                                   'robot_model_name': scene['robot']['name']}]),
    ]
    if cameras:
        actions.append(Node(package='galbot_s1_control', executable='camera_relay.py',
                            name='galbot_camera_relay', output='screen', parameters=[common]))
    if web:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('galbot_s1_web'), 'launch', 'web.launch.py')),
            launch_arguments={'port': arg('web_port'), 'rosbridge_port': arg('rosbridge_port')}.items()))
    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2', parameters=[common],
                            arguments=['-d', os.path.join(get_package_share_directory('galbot_s1_description'),
                                                          'rviz', 'galbot_s1_ros2.rviz'), '-f', 'odom']))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI (false = headless server)'),
        DeclareLaunchArgument('cameras', default_value='true', description='head and wrist cameras'),
        DeclareLaunchArgument('web', default_value='true', description='rosbridge + web UI'),
        DeclareLaunchArgument('web_port', default_value='8000'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('world', default_value='', description='SDF world (default: table_scene.sdf)'),
        DeclareLaunchArgument('scene_file', default_value='', description='scene yaml (default: config/scene.yaml)'),
        OpaqueFunction(function=launch_setup),
    ])
