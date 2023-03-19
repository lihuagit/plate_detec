from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_launch_description():

    # params file path
    params_file = os.path.join(
        get_package_share_directory('bringup'), 'config', 'params.yaml')

    # load params for composable node
    with open(params_file, 'r') as f:
        camera_params = yaml.safe_load(f)['/mv_camera']['ros__parameters']
    with open(params_file, 'r') as f:
        detector_params = yaml.safe_load(f)['/armor_detector']['ros__parameters']
    with open(params_file, 'r') as f:
        tracker_params = yaml.safe_load(f)['/armor_tracker_node']['ros__parameters']
    with open(params_file, 'r') as f:
        serial_params = yaml.safe_load(f)['/serial_driver']['ros__parameters']
        
    detector_node = ComposableNode(
        package='armor_detector',
        plugin='armor_auto_aim::DetectorNode',
        name='armor_detector',
        parameters=[detector_params],
        extra_arguments=[{'use_intra_process_comms': True}]
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
    
    tracker_node = Node(
        package='armor_tracker',
        executable='armor_tracker_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[tracker_params],
    )
    
    serial_node = Node(
        package='serial',
        executable='serial_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[serial_params],
    )
    

    return LaunchDescription([
        mv_camera_detector_container,
        tracker_node,
        serial_node
    ])