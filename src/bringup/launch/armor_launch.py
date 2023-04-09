from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node 

from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PythonExpression, LaunchConfiguration
from launch.conditions import IfCondition
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_launch_description():

    use_serial = LaunchConfiguration('use_serial')
    
    declare_use_serial_cmd = DeclareLaunchArgument(
        'use_serial',
        default_value='True',
        description='Whether use serial port')

    # params file path
    params_file = os.path.join(
        get_package_share_directory('bringup'), 'config', 'params.yaml')

    # load params for composable node
    with open(params_file, 'r') as f:
        camera_params = yaml.safe_load(f)['/mv_camera']['ros__parameters']
    with open(params_file, 'r') as f:
        detector_params = yaml.safe_load(f)['/armor_detector']['ros__parameters']
    with open(params_file, 'r') as f:
        processor_params = yaml.safe_load(f)['/armor_processor']['ros__parameters']
    with open(params_file, 'r') as f:
        serial_params = yaml.safe_load(f)['/lc_serial_driver']['ros__parameters']

    # robot_description
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_description'), 'urdf', 'gimbal_b3.urdf.xacro')])
        
    detector_node = ComposableNode(
        package='armor_detector',
        plugin='rm_auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[detector_params],
        extra_arguments=[{'use_intra_process_comms': True}],
        # arguments=['--ros-args', '--log-level', 'armor_detector:=DEBUG'],
    )

    mv_camera_detector_container = ComposableNodeContainer(
        name='camera_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='mindvision_camera',
                plugin='mindvision_camera::MVCameraNode',
                name='camera_node',
                parameters=[camera_params, {'use_sensor_data_qos': True}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            detector_node
        ],
        output='screen',
    )
    
    processor_node = Node(
        package='armor_processor',
        executable='armor_processor_node',
        output='screen',
        emulate_tty=True,
        parameters=[processor_params],
        arguments=['--ros-args', '--log-level', 'armor_processor:=DEBUG'],
    )
    
    serial_node = Node(
        package='lc_serial',
        executable='serial_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[serial_params],
        condition=IfCondition(use_serial)
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'publish_frequency': 1000.0}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'rate': 600}],
        condition=IfCondition(PythonExpression(["not ", use_serial]))
    )

    return LaunchDescription([
        declare_use_serial_cmd,
        
        mv_camera_detector_container,
        processor_node,
        serial_node,
        robot_state_publisher,
        joint_state_publisher,
    ])