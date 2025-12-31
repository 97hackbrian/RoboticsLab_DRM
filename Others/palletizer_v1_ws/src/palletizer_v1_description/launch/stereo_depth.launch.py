#!/usr/bin/env python3
"""
Stereo Depth Image Processing Launch File

This launch file starts stereo image processing nodes to generate:
- Disparity image from stereo camera pair
- Point cloud from stereo disparity
- Depth image from point cloud

The stereo_camera_info_publisher node fixes the CameraInfo messages by
adding the correct baseline (16cm) to the right camera's projection matrix.

Input topics (from Gazebo):
- /camera/left/image_raw
- /camera/left/camera_info  
- /camera/right/image_raw
- /camera/right/camera_info

Output topics:
- /camera/disparity (stereo_msgs/DisparityImage)
- /camera/depth/points (sensor_msgs/PointCloud2)
- /camera/depth/image (sensor_msgs/Image - depth image)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    show_stereo_view = LaunchConfiguration('show_stereo_view', default='true')
    baseline = LaunchConfiguration('baseline', default='0.16')  # 16cm
    
    # Stereo camera namespace
    camera_namespace = '/camera'
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            'show_stereo_view',
            default_value='true',
            description='Show stereo viewer window'
        ),
        
        DeclareLaunchArgument(
            'baseline',
            default_value='0.16',
            description='Stereo camera baseline in meters'
        ),
        
        LogInfo(msg='\n========================================================'),
        LogInfo(msg='Stereo Depth Processing Starting...'),
        LogInfo(msg='--------------------------------------------------------'),
        LogInfo(msg='Baseline: 0.16 m (16 cm)'),
        LogInfo(msg='--------------------------------------------------------'),
        LogInfo(msg='Input Topics:'),
        LogInfo(msg='  - /camera/left/image_raw'),
        LogInfo(msg='  - /camera/left/camera_info'),
        LogInfo(msg='  - /camera/right/image_raw'),
        LogInfo(msg='  - /camera/right/camera_info'),
        LogInfo(msg='--------------------------------------------------------'),
        LogInfo(msg='Output Topics:'),
        LogInfo(msg='  - /camera/disparity (stereo_msgs/DisparityImage)'),
        LogInfo(msg='  - /camera/depth/points (sensor_msgs/PointCloud2)'),
        LogInfo(msg='  - /camera/depth/image (sensor_msgs/Image)'),
        LogInfo(msg='========================================================\n'),
        
        # ============================================================
        # STEREO CAMERA INFO PUBLISHER
        # Fixes CameraInfo by adding baseline to right camera P matrix
        # ============================================================
        
        Node(
            package='palletizer_v1_description',
            executable='stereo_camera_info_publisher',
            name='stereo_camera_info_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'baseline': 0.16,  # 16cm baseline
                'left_camera_info_in': '/camera/left/camera_info',
                'right_camera_info_in': '/camera/right/camera_info',
                'left_camera_info_out': '/stereo/left/camera_info',
                'right_camera_info_out': '/stereo/right/camera_info',
                'left_frame_id': 'camera_left_optical_frame',
                'right_frame_id': 'camera_right_optical_frame',
            }],
        ),
        
        # ============================================================
        # IMAGE REPUBLISHERS
        # Republish images to /stereo namespace for stereo_image_proc
        # ============================================================
        
        # Left image relay
        Node(
            package='image_transport',
            executable='republish',
            name='left_image_republish',
            arguments=['raw', 'raw'],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('in', '/camera/left/image_raw'),
                ('out', '/stereo/left/image_raw'),
            ],
        ),
        
        # Right image relay
        Node(
            package='image_transport',
            executable='republish',
            name='right_image_republish',
            arguments=['raw', 'raw'],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('in', '/camera/right/image_raw'),
                ('out', '/stereo/right/image_raw'),
            ],
        ),
        
        # ============================================================
        # STEREO IMAGE PROCESSING - DISPARITY
        # Uses stereo_image_proc to compute disparity from stereo pair
        # ============================================================
        
        # Disparity Node - Computes disparity image from stereo pair
        Node(
            package='stereo_image_proc',
            executable='disparity_node',
            name='stereo_disparity',
            namespace='/stereo',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                # Stereo matching parameters
                'stereo_algorithm': 0,  # 0=BM, 1=SGBM
                'prefilter_size': 9,
                'prefilter_cap': 31,
                'correlation_window_size': 21,
                'min_disparity': 0,
                'disparity_range': 128,  # Increased for larger baseline
                'texture_threshold': 10,
                'speckle_size': 100,
                'speckle_range': 4,
                'uniqueness_ratio': 15.0,
                'queue_size': 10,
                'approximate_sync': True,
            }],
            remappings=[
                ('left/image_rect', 'left/image_raw'),
                ('left/camera_info', 'left/camera_info'),
                ('right/image_rect', 'right/image_raw'),
                ('right/camera_info', 'right/camera_info'),
            ]
        ),
        
        # Point Cloud Node - Generates colored point cloud
        Node(
            package='stereo_image_proc',
            executable='point_cloud_node',
            name='stereo_point_cloud',
            namespace='/stereo',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'approximate_sync': True,
                'queue_size': 10,
                'use_color': True,
            }],
            remappings=[
                ('left/image_rect_color', 'left/image_raw'),
                ('left/camera_info', 'left/camera_info'),
                ('right/camera_info', 'right/camera_info'),
                ('points2', '/camera/depth/points'),
            ]
        ),
        
        # ============================================================
        # DEPTH IMAGE FROM POINT CLOUD
        # ============================================================
        
        Node(
            package='rtabmap_util',
            executable='pointcloud_to_depthimage',
            name='pointcloud_to_depth',
            namespace=camera_namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'decimation': 1,
                'fixed_frame_id': '',
                'fill_holes_size': 0,
                'min_range': 0.0,
                'max_range': 0.0,
            }],
            remappings=[
                ('cloud', '/camera/depth/points'),
                ('camera_info', '/stereo/left/camera_info'),
                ('image', 'depth/image'),
                ('image_raw', 'depth/image_raw'),
            ]
        ),
        
        # ============================================================
        # POINT CLOUD TRANSFORM
        # Transform from optical frame to robot base frame
        # Transformation: x=z, y=-x, z=-y (based on actual TF)
        # ============================================================
        
        Node(
            package='palletizer_v1_description',
            executable='pointcloud_transform_publisher',
            name='pointcloud_transform',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/camera/depth/points',
                'output_topic': '/camera/depth/points_robot',
                'output_frame_id': 'base_footprint',  # Use robot base frame
            }],
        ),
        
        # NOTA: Disparity está disponible en /stereo/disparity
        # Point cloud original: /camera/depth/points (frame óptico)
        # Point cloud transformado: /camera/depth/points_robot (frame robot)
        # Depth image está disponible en /camera/depth/image
        
        # ============================================================
        # STEREO VIEW FOR DEBUGGING
        # ============================================================
        
        Node(
            package='image_view',
            executable='stereo_view',
            name='stereo_view',
            namespace='/stereo',
            output='screen',
            condition=IfCondition(show_stereo_view),
            parameters=[{
                'use_sim_time': use_sim_time,
                'approximate_sync': True,
                'queue_size': 10,
            }],
            remappings=[
                ('stereo/left/image_raw', 'left/image_raw'),
                ('stereo/right/image_raw', 'right/image_raw'),
                ('stereo/disparity', 'disparity'),
            ]
        ),
    ])
