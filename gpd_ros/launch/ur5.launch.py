from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gpd_ros',
            executable='detect_grasps',
            name='detect_grasps',
            output='screen',
            parameters=[{
                # If sequential importance sampling is used (default: false)
                # 'use_importance_sampling': False,

                # What type of point cloud is used and the ROS topic it comes from
                'cloud_type': 0,  # 0: PointCloud2, 1: CloudIndexed, 2: CloudSamples
                'cloud_topic': '/rgbd_camera/points',

                # Optional: ROS topic that the samples come from
                'samples_topic': '',

                # Filepath to the configuration file for GPD
                'config_file': '/home/manjeet/gpd/cfg/ros_eigen_params.cfg',

                # RViz topic to publish grasps
                'rviz_topic': 'plot_grasps',

                'use_sim_time': True
            }]
        )
    ])