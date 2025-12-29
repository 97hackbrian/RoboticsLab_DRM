"""
RTAB-Map Stereo Visual Odometry Launch File
Provides stereo odometry for palletizer_v1 robot using RTAB-Map's stereo_odometry node.
Publishes odom -> base_footprint TF and /odom topic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # RTAB-Map stereo odometry parameters
    # These are tuned for simulated environment
    rtabmap_parameters = {
        # Frame configuration
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'wait_for_transform': 0.2,
        
        # Synchronization
        'approx_sync': True,
        'approx_sync_max_interval': 0.05,  # 50ms max interval between left/right images
        'queue_size': 10,
        
        # Odometry parameters
        'Odom/Strategy': '0',  # 0=Frame-to-Map, 1=Frame-to-Frame
        'Odom/GuessMotion': 'true',
        'Odom/ResetCountdown': '1',
        'Vis/EstimationType': '1',  # 0=3D->3D, 1=3D->2D (PnP)
        'Vis/MaxDepth': '10.0',
        'Vis/MinInliers': '15',
        'Vis/InlierDistance': '0.1',
        
        # Feature detection (GFTT for better performance in simulation)
        'Vis/FeatureType': '6',  # 6=GFTT
        'Vis/MaxFeatures': '500',
        'GFTT/MinDistance': '5',
        'GFTT/QualityLevel': '0.001',
        
        # Stereo matching parameters
        'Stereo/WinWidth': '15',
        'Stereo/WinHeight': '3',
        'Stereo/MaxLevel': '5',
        'Stereo/Iterations': '30',
        'Stereo/MinDisparity': '0',
        'Stereo/MaxDisparity': '128',
        
        # Use simulation time
        'use_sim_time': use_sim_time,
    }
    
    stereo_odometry_node = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=[
            # Stereo camera topics
            ('left/image_rect', '/camera/left/image_raw'),
            ('right/image_rect', '/camera/right/image_raw'),
            ('left/camera_info', '/camera/left/camera_info'),
            ('right/camera_info', '/camera/right/camera_info'),
            # Output
            ('odom', '/odom'),
        ],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        stereo_odometry_node,
    ])
