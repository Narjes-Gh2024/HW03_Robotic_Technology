from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os


def generate_launch_description():
    robot_desc_dir = get_package_share_directory('robot_description')
    robot_slam_dir = get_package_share_directory('robot_slam')

    world = os.path.join(robot_desc_dir, 'world', 'depot.sdf')
    urdf_file = os.path.join(robot_desc_dir, 'src', 'description', 'robot.urdf')
    rviz_config_file = os.path.join(robot_slam_dir, 'rviz', 'slam.rviz')
    gz_bridge_config = os.path.join(robot_desc_dir, 'config', 'gz_bridge.yaml')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(robot_desc_dir, 'world'),
            str(Path(robot_desc_dir).parent.resolve())
        ])
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world]}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gz_bridge_config,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc}
        ]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-topic', '/robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.9',
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    lidar_frame_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_alias_tf',
        output='screen',
        arguments=[
            '--frame-id', 'robot/base_link',
            '--child-frame-id', 'rplidar_c1',
            '--x', '0', '--y', '0', '--z', '0.4',
            '--roll', '0', '--pitch', '0', '--yaw',
        ],
        parameters=[{'use_sim_time': True}]
    )

    frame_id_converter_node = Node(
        package='robot_localization',
        executable='frame_id_converter',
        name='frame_id_converter_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    motor_command_node = Node(
        package='robot_localization',
        executable='motor_command_node',
        name='motor_command_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    ) 

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/ekf/odom',
        description='Odometry topic name'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Laser scan topic name'
    )

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame ID'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame ID'
    )

    map_resolution_arg = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.05',
        description='Map resolution in meters per cell'
    )

    map_width_arg = DeclareLaunchArgument(
        'map_width',
        default_value='50.0',
        description='Map width in meters'
    )

    map_height_arg = DeclareLaunchArgument(
        'map_height',
        default_value='50.0',
        description='Map height in meters'
    )

    map_origin_x_arg = DeclareLaunchArgument(
        'map_origin_x',
        default_value='-25.0',
        description='Map origin X coordinate'
    )

    map_origin_y_arg = DeclareLaunchArgument(
        'map_origin_y',
        default_value='-25.0',
        description='Map origin Y coordinate'
    )

    use_scan_matching_arg = DeclareLaunchArgument(
        'use_scan_matching',
        default_value='true',
        description='Enable scan-to-map matching for pose correction'
    )

    scan_matching_iterations_arg = DeclareLaunchArgument(
        'scan_matching_iterations',
        default_value='20',
        description='Maximum iterations for scan matching'
    )

    map_update_interval_arg = DeclareLaunchArgument(
        'map_update_interval',
        default_value='1.0',
        description='Interval between map publications in seconds'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.expanduser('~/maps'),
        description='Directory for saving map files'
    )

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='slam_map',
        description='Base name for saved map files'
    )

    auto_save_arg = DeclareLaunchArgument(
        'auto_save',
        default_value='false',
        description='Enable automatic periodic map saving'
    )

    auto_save_interval_arg = DeclareLaunchArgument(
        'auto_save_interval',
        default_value='60.0',
        description='Interval for auto-saving in seconds'
    )

    slam_node = Node(
        package='robot_slam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': LaunchConfiguration('odom_topic'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'map_frame': LaunchConfiguration('map_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width': LaunchConfiguration('map_width'),
            'map_height': LaunchConfiguration('map_height'),
            'map_origin_x': LaunchConfiguration('map_origin_x'),
            'map_origin_y': LaunchConfiguration('map_origin_y'),
            'use_scan_matching': LaunchConfiguration('use_scan_matching'),
            'scan_matching_iterations': LaunchConfiguration('scan_matching_iterations'),
            'map_update_interval': LaunchConfiguration('map_update_interval'),
        }]
    )

    map_saver_node = Node(
        package='robot_slam',
        executable='map_saver_node',
        name='map_saver_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_topic': '/map',
            'output_dir': LaunchConfiguration('output_dir'),
            'map_name': LaunchConfiguration('map_name'),
            'auto_save': LaunchConfiguration('auto_save'),
            'auto_save_interval': LaunchConfiguration('auto_save_interval'),
        }]
    )

    return LaunchDescription([
        odom_topic_arg,
        scan_topic_arg,
        map_frame_arg,
        odom_frame_arg,
        base_frame_arg,
        map_resolution_arg,
        map_width_arg,
        map_height_arg,
        map_origin_x_arg,
        map_origin_y_arg,
        use_scan_matching_arg,
        scan_matching_iterations_arg,
        map_update_interval_arg,
        output_dir_arg,
        map_name_arg,
        auto_save_arg,
        auto_save_interval_arg,    
        gz_resource_path,
        gz_sim,
        bridge,
        start_robot_state_publisher_cmd,
        spawn_entity,
        lidar_frame_alias_tf,
        rviz_node,
        frame_id_converter_node,
        ekf_node,
        motor_command_node,
        slam_node,
        map_saver_node,
    ])
