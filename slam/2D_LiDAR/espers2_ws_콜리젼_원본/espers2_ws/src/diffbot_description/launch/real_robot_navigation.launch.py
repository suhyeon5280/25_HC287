import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('diffbot_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # URDF 파일 로드
    urdf_file = os.path.join(pkg_share, 'urdf', 'driving_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )

    # 하드웨어 드라이버 노드들
    arduino_bridge_node = Node(
        package='arduino_bridge',
        executable='arduino_bridge.py',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'port': '/dev/arduino',
            'baudrate': 115200
        }]
    )

    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': '/dev/lidar',
            'serial_baudrate': 115200,
            'frame_id': 'lidar',
            'angle_compensate': True,
        }],
        output='screen'
    )

    laser_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        parameters=[os.path.join(pkg_share, 'config', 'laser_filter.yaml')],
        remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')],
        output='screen'
    )

    # 위치 추정(Localization) 노드
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_share, 'config', 'localization_params.yaml'),
            'use_sim_time': 'false'
        }.items()
    )

    # 내비게이션(Nav2) 노드
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_share, 'config', 'map.yaml') # ★★★ 이 줄이 다시 필요합니다.

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'map': map_file  # ★★★ 이 줄을 다시 추가합니다.
        }.items()
    )
    
    # 시각화(RViz) 노드
    rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        arduino_bridge_node,
        sllidar_node,
        laser_filter,
        localization_launch,
        navigation_launch,
        rviz_node
    ])















# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node

# def generate_launch_description():
#     pkg_share = get_package_share_directory('diffbot_description')
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')
#     gazebo_ros_dir = get_package_share_directory('gazebo_ros')

#     # --- 1. 파라미터 및 파일 경로 설정 ---
#     urdf_file = os.path.join(pkg_share, 'urdf', 'driving_robot_play.urdf')
#     with open(urdf_file, 'r') as infp:
#         robot_desc = infp.read()
    
#     nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
#     map_file = os.path.join(pkg_share, 'config', 'map.yaml')
#     rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
#     world_file = os.path.join(pkg_share, 'worlds', 'test.world') # 가제보 월드 파일 경로

#     # --- 2. use_sim_time 파라미터 설정 ---
#     use_sim_time = True # True로 설정

#     # --- 3. 핵심 노드 정의 ---

    
#     # 가제보 실행
#     gazebo_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
#         ),
#         launch_arguments={'world': world_file}.items()
#     )

#     # Robot State Publisher (URDF 기반 TF 발행)
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'robot_description': robot_desc,
#             'use_sim_time': use_sim_time
#         }]
#     )

#     # 가제보에 로봇 모델 스폰
#     spawn_entity_node = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-topic', 'robot_description', '-entity', 'diffbot'],
#         output='screen'
#     )


#     laser_filter = Node(
#         package='laser_filters',
#         executable='scan_to_scan_filter_chain',
#         name='scan_filter',
#         parameters=[os.path.join(pkg_share, 'config', 'laser_filter.yaml')],
#         remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')],
#         output='screen'
#     )

#     # 내비게이션(Nav2) 스택 실행
#     navigation_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
#         ),
#         launch_arguments={
#             'use_sim_time': str(use_sim_time), # 문자열로 변환
#             'params_file': nav2_params_file,
#             'map': map_file
#         }.items()
#     )
    
#     # RViz 실행
#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         arguments=['-d', rviz_config],
#         parameters=[{'use_sim_time': use_sim_time}],
#         output='screen'
#     )

#     # --- 4. 런치 파일 구성 ---
#     return LaunchDescription([
#         gazebo_launch,
#         robot_state_publisher_node,
#         spawn_entity_node,
#         navigation_launch,
#         laser_filter,
#         rviz_node
#     ])