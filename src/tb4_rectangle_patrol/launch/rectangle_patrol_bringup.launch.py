from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    pkg_patrol = get_package_share_directory('tb4_rectangle_patrol')
    pkg_tb4_nav = get_package_share_directory('turtlebot4_navigation')
    pkg_tb4_description = get_package_share_directory('turtlebot4_description')
    pkg_tb4_ignition = get_package_share_directory('turtlebot4_ignition_bringup')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    ARGUMENTS = [
        DeclareLaunchArgument(
            'start_sim',
            default_value='true',
            choices=['true', 'false'],
            description='Start the matching rectangle Ignition simulation.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use simulation time.'),
        DeclareLaunchArgument(
            'model',
            default_value='lite',
            choices=['standard', 'lite'],
            description='TurtleBot4 model.'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace.'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([pkg_patrol, 'config', 'nav2_rectangle.yaml']),
            description='Nav2 parameters file.'),
        DeclareLaunchArgument(
            'localization_params',
            default_value=PathJoinSubstitution([pkg_tb4_nav, 'config', 'localization.yaml']),
            description='Localization parameters file.'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([pkg_patrol, 'maps', 'rectangle_2m_x_1_5m.yaml']),
            description='Map yaml file.'),
        DeclareLaunchArgument(
            'patrol_params',
            default_value=PathJoinSubstitution([pkg_patrol, 'config', 'rectangle_patrol.yaml']),
            description='Rectangle patrol parameters file.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([pkg_patrol, 'rviz', 'rectangle_patrol.rviz']),
            description='RViz configuration file.'),
        DeclareLaunchArgument(
            'initial_x',
            default_value='1.8',
            description='Initial robot x position in map/simulation coordinates.'),
        DeclareLaunchArgument(
            'initial_y',
            default_value='0.2',
            description='Initial robot y position in map/simulation coordinates.'),
        DeclareLaunchArgument(
            'initial_yaw',
            default_value='1.57079632679',
            description='Initial robot yaw; +pi/2 faces the top-right corner from bottom-right.'),
    ]

    start_sim = LaunchConfiguration('start_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    model = LaunchConfiguration('model')
    nav2_params = LaunchConfiguration('nav2_params')
    localization_params = LaunchConfiguration('localization_params')
    map_yaml = LaunchConfiguration('map')
    patrol_params = LaunchConfiguration('patrol_params')
    rviz_config = LaunchConfiguration('rviz_config')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')

    rectangle_world = PathJoinSubstitution(
        [pkg_patrol, 'worlds', 'rectangle_2m_x_1_5m.sdf'])

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])),
        launch_arguments={
            'ign_args': [rectangle_world, ' -r -v 4'],
        }.items(),
        condition=IfCondition(start_sim))

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        condition=IfCondition(start_sim))

    sim_robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_ignition, 'launch', 'turtlebot4_spawn.launch.py'])),
        launch_arguments={
            'model': model,
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'rviz': 'false',
            'localization': 'false',
            'slam': 'false',
            'nav2': 'false',
            'x': initial_x,
            'y': initial_y,
            'z': '0.0',
            'yaw': initial_yaw,
        }.items(),
        condition=IfCondition(start_sim))

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_description, 'launch', 'robot_description.launch.py'])),
        launch_arguments={
            'model': model,
            'use_sim_time': use_sim_time,
            'namespace': namespace,
        }.items(),
        condition=UnlessCondition(start_sim))

    rplidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rplidar_stf',
        output='screen',
        arguments=[
            '0', '0', '0', '0', '0', '0.0',
            'rplidar_link', 'turtlebot4/rplidar_link/rplidar'],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        condition=UnlessCondition(start_sim))

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_nav, 'launch', 'localization.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'params': localization_params,
            'map': map_yaml,
        }.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_nav, 'launch', 'nav2.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'params_file': nav2_params,
        }.items())

    initial_pose = Node(
        package='tb4_rectangle_patrol',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            patrol_params,
            {
                'use_sim_time': use_sim_time,
                'initial_x': initial_x,
                'initial_y': initial_y,
                'initial_yaw': initial_yaw,
            },
        ])

    patrol = Node(
        package='tb4_rectangle_patrol',
        executable='rectangle_patrol',
        name='rectangle_patrol',
        namespace=namespace,
        output='screen',
        parameters=[patrol_params, {'use_sim_time': use_sim_time}])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        *ARGUMENTS,
        ignition,
        clock_bridge,
        sim_robot_spawn,
        robot_description,
        rplidar_static_tf,
        localization,
        nav2,
        initial_pose,
        patrol,
        rviz,
    ])
