import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    pkg_share = get_package_share_directory('diffbot_description')
    
    sllidar_launch_path = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a1_launch.py' 
    )

    
    urdf_file = os.path.join(pkg_share, 'urdf', 'driving_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False  
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}] 
    )


    sllidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch_path),
        launch_arguments={
            'use_sim_time': 'false',
            'serial_port': '/dev/lidar'     
        }.items()
    )

    laser_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        parameters=[os.path.join(pkg_share, 'config', 'laser_filter.yaml')],
        remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')],
        output='screen'
    )


    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_share, 'config', 'real_robot_mapping_params.yaml'),
            'use_sim_time': 'false' 
        }.items()
    )


    rviz_config = os.path.join(pkg_share, 'rviz', 'diffbot2.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}], 
        output='screen'
    )

    return LaunchDescription([
        sllidar_driver,
        robot_state_publisher,
        joint_state_publisher,
        laser_filter,
        slam_launch,
        rviz
    ])






# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node

# def generate_launch_description():

#     pkg_share = get_package_share_directory('diffbot_description')
    

#     sllidar_launch_path = os.path.join(
#         get_package_share_directory('sllidar_ros2'),
#         'launch',
#         'sllidar_a1_launch.py' 
#     )


#     urdf_file = os.path.join(pkg_share, 'urdf', 'driving_robot.urdf')
#     with open(urdf_file, 'r') as infp:
#         robot_desc = infp.read()


#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'robot_description': robot_desc,
#             'use_sim_time': False  
#         }]
#     )


#     joint_state_publisher = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         name='joint_state_publisher',
#         parameters=[{'use_sim_time': False}] 
#     )


#     sllidar_driver = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(sllidar_launch_path),
#         launch_arguments={
#             'use_sim_time': 'false',
#             'serial_port': '/dev/lidar'
#         }.items()
#     )


#     slam_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
#         ),
#         launch_arguments={
#             'slam_params_file': os.path.join(pkg_share, 'config', 'real_robot_mapping_params.yaml'),
#             'use_sim_time': 'false' 
#         }.items()
#     )


#     rviz_config = os.path.join(pkg_share, 'rviz', 'diffbot2.rviz')
#     rviz = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         arguments=['-d', rviz_config],
#         parameters=[{'use_sim_time': False}], 
#         output='screen'
#     )

#     return LaunchDescription([
#         sllidar_driver,
#         robot_state_publisher,
#         joint_state_publisher,
#         slam_launch,
#         rviz
#     ])