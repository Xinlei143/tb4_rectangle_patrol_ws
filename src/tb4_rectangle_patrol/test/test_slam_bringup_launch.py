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
    assert "'turtlebot4_spawn.launch.py'" in launch_source
    assert "'x': initial_x" in launch_source
    assert "'y': initial_y" in launch_source
    assert "'yaw': initial_yaw" in launch_source


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
