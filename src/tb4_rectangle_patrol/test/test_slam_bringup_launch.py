from pathlib import Path


def test_bringup_uses_static_map_localization():
    launch_path = (
        Path(__file__).parents[1]
        / 'launch'
        / 'rectangle_patrol_bringup.launch.py'
    )
    launch_source = launch_path.read_text(encoding='utf-8')

    assert "'localization.launch.py'" in launch_source
    assert "'slam.launch.py'" not in launch_source
    assert "DeclareLaunchArgument(\n            'map'," in launch_source
    assert "'map': map_yaml" in launch_source


def test_bringup_launches_robot_description_and_lidar_static_tf():
    launch_path = (
        Path(__file__).parents[1]
        / 'launch'
        / 'rectangle_patrol_bringup.launch.py'
    )
    launch_source = launch_path.read_text(encoding='utf-8')

    assert "get_package_share_directory('turtlebot4_description')" in launch_source
    assert "'robot_description.launch.py'" in launch_source
    assert "executable='static_transform_publisher'" in launch_source
    assert "'rplidar_link'" in launch_source
    assert "'turtlebot4/rplidar_link/rplidar'" in launch_source


def test_bringup_defaults_to_simulation_time():
    launch_path = (
        Path(__file__).parents[1]
        / 'launch'
        / 'rectangle_patrol_bringup.launch.py'
    )
    launch_source = launch_path.read_text(encoding='utf-8')

    assert "default_value='true'" in launch_source


def test_bringup_can_start_matching_rectangle_simulation():
    launch_path = (
        Path(__file__).parents[1]
        / 'launch'
        / 'rectangle_patrol_bringup.launch.py'
    )
    launch_source = launch_path.read_text(encoding='utf-8')

    assert "DeclareLaunchArgument(\n            'start_sim'," in launch_source
    assert "'rectangle_2m_x_1_5m.sdf'" in launch_source
    assert "'rectangle_turtlebot4_spawn.launch.py'" in launch_source
    assert "'x': initial_x" in launch_source
    assert "'y': initial_y" in launch_source
    assert "'yaw': initial_yaw" in launch_source
    assert "'dock_offset': dock_offset" in launch_source
    assert "'dock_yaw': dock_yaw" in launch_source


def test_simulation_delays_nav2_and_patrol_startup():
    launch_path = (
        Path(__file__).parents[1]
        / 'launch'
        / 'rectangle_patrol_bringup.launch.py'
    )
    launch_source = launch_path.read_text(encoding='utf-8')

    assert 'TimerAction' in launch_source
    assert "DeclareLaunchArgument(\n            'nav2_start_delay'," in launch_source
    assert "default_value='8.0'" in launch_source
    assert 'delayed_sim_navigation = TimerAction(' in launch_source
    assert 'period=nav2_start_delay' in launch_source
    assert 'condition=IfCondition(start_sim)' in launch_source
    assert 'actions=[nav2_sim, patrol_sim]' in launch_source


def test_real_robot_navigation_is_not_delayed():
    launch_path = (
        Path(__file__).parents[1]
        / 'launch'
        / 'rectangle_patrol_bringup.launch.py'
    )
    launch_source = launch_path.read_text(encoding='utf-8')

    assert 'nav2_real = IncludeLaunchDescription(' in launch_source
    assert 'patrol_real = Node(' in launch_source
    assert 'condition=UnlessCondition(start_sim))' in launch_source


def test_custom_spawn_places_dock_behind_robot():
    spawn_path = (
        Path(__file__).parents[1]
        / 'launch'
        / 'rectangle_turtlebot4_spawn.launch.py'
    )
    spawn_source = spawn_path.read_text(encoding='utf-8')

    assert "RotationalOffsetX(-0.157, yaw)" in spawn_source
    assert "RotationalOffsetY(-0.157, yaw)" in spawn_source
    assert "'-Y', dock_yaw" in spawn_source
    assert "'standard_dock_description'" in spawn_source


def test_patrol_waits_for_amcl_in_static_map_bringup():
    package_path = Path(__file__).parents[1]
    config_source = (
        package_path / 'config' / 'rectangle_patrol.yaml'
    ).read_text(encoding='utf-8')
    node_source = (
        package_path / 'tb4_rectangle_patrol' / 'rectangle_patrol_node.py'
    ).read_text(encoding='utf-8')

    assert 'localizer: amcl' in config_source
    assert "self.navigator.declare_parameter('localizer', 'amcl')" in node_source
    assert "self.navigator._waitForNodeToActivate(self.localizer)" in node_source
    assert 'self._wait_for_localized_pose()' in node_source
    assert 'self.navigator.waitUntilNav2Active(localizer=self.localizer)' not in node_source


def test_bringup_launches_independent_initial_pose_publisher():
    launch_path = (
        Path(__file__).parents[1]
        / 'launch'
        / 'rectangle_patrol_bringup.launch.py'
    )
    launch_source = launch_path.read_text(encoding='utf-8')

    assert "executable='initial_pose_publisher'" in launch_source
    assert "package='tb4_rectangle_patrol'" in launch_source


def test_initial_pose_publisher_does_not_wait_for_sim_ready():
    package_path = Path(__file__).parents[1]
    initial_pose_source = (
        package_path / 'tb4_rectangle_patrol' / 'initial_pose_publisher.py'
    ).read_text(encoding='utf-8')
    config_source = (
        package_path / 'config' / 'rectangle_patrol.yaml'
    ).read_text(encoding='utf-8')

    assert 'wait_for_sim_ready: true' in config_source
    assert 'sim_ready_timeout_sec: 45.0' in config_source
    assert 'wait_for_sim_ready(' not in initial_pose_source
    assert 'if not self.map_received:' in initial_pose_source


def test_patrol_waits_for_sim_ready_before_motion():
    package_path = Path(__file__).parents[1]
    patrol_source = (
        package_path / 'tb4_rectangle_patrol' / 'rectangle_patrol_node.py'
    ).read_text(encoding='utf-8')

    assert patrol_source.index('wait_for_sim_ready(') < patrol_source.index('self._simple_undock()')


def test_sim_ready_waiter_uses_global_clock_and_optional_motion_topics():
    source = (
        Path(__file__).parents[1] / 'tb4_rectangle_patrol' / 'sim_ready.py'
    ).read_text(encoding='utf-8')

    assert "'/clock'" in source
    assert 'require_scan=True' in source
    assert 'require_odom=True' in source
    assert 'require_controller=True' in source


def test_patrol_uses_simple_forward_undock_in_simulation():
    package_path = Path(__file__).parents[1]
    config_source = (
        package_path / 'config' / 'rectangle_patrol.yaml'
    ).read_text(encoding='utf-8')
    node_source = (
        package_path / 'tb4_rectangle_patrol' / 'rectangle_patrol_node.py'
    ).read_text(encoding='utf-8')

    assert 'undock_mode: drive_forward' in config_source
    assert 'undock_distance: 0.1' in config_source
    assert 'self._simple_undock()' in node_source


def test_default_map_points_to_rectangle_asset():
    launch_path = (
        Path(__file__).parents[1]
        / 'launch'
        / 'rectangle_patrol_bringup.launch.py'
    )
    launch_source = launch_path.read_text(encoding='utf-8')

    assert "'maps', 'rectangle_2m_x_1_5m.yaml'" in launch_source


def test_default_nav2_params_use_rectangle_tuned_inflation():
    package_path = Path(__file__).parents[1]
    launch_source = (
        package_path / 'launch' / 'rectangle_patrol_bringup.launch.py'
    ).read_text(encoding='utf-8')
    nav2_source = (
        package_path / 'config' / 'nav2_rectangle.yaml'
    ).read_text(encoding='utf-8')

    assert "'config', 'nav2_rectangle.yaml'" in launch_source
    assert nav2_source.count('inflation_radius: 0.20') == 2
    assert nav2_source.count('cost_scaling_factor: 6.0') == 2
