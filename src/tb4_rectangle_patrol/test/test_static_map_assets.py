from pathlib import Path
import math

import yaml

from tb4_rectangle_patrol.initial_pose_publisher import make_initial_pose
from tb4_rectangle_patrol.map_bounds import load_free_space_bounds
from tb4_rectangle_patrol.rectangle_geometry import compute_rectangle_waypoints


def test_rectangle_map_yaml_matches_requested_geometry():
    map_yaml = (
        Path(__file__).parents[1]
        / 'maps'
        / 'rectangle_2m_x_1_5m.yaml'
    )
    metadata = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))

    assert metadata['image'] == 'rectangle_2m_x_1_5m.pgm'
    assert metadata['resolution'] == 0.05
    assert metadata['origin'] == [-0.2, -0.2, 0.0]
    assert metadata['negate'] == 0
    assert metadata['occupied_thresh'] == 0.65
    assert metadata['free_thresh'] == 0.25


def test_rectangle_map_image_has_expected_size():
    map_image = (
        Path(__file__).parents[1]
        / 'maps'
        / 'rectangle_2m_x_1_5m.pgm'
    )
    tokens = map_image.read_text(encoding='ascii').split()

    assert tokens[:4] == ['P2', '48', '38', '255']
    assert len(tokens[4:]) == 48 * 38


def test_rectangle_sim_world_keeps_turtlebot4_bridge_world_name():
    world = (
        Path(__file__).parents[1]
        / 'worlds'
        / 'rectangle_2m_x_1_5m.sdf'
    ).read_text(encoding='utf-8')

    assert '<world name="warehouse">' in world


def test_default_initial_pose_is_safe_right_bottom_corner():
    config_yaml = (
        Path(__file__).parents[1]
        / 'config'
        / 'rectangle_patrol.yaml'
    )
    config = yaml.safe_load(config_yaml.read_text(encoding='utf-8'))
    params = config['rectangle_patrol']['ros__parameters']

    assert params['initial_pose_enabled'] is True
    assert params['initial_x'] == 1.8
    assert params['initial_y'] == 0.2
    assert math.isclose(params['initial_yaw'], math.pi / 2.0)


def test_initial_pose_message_uses_map_frame_and_zero_timestamp():
    pose = make_initial_pose(x=1.8, y=0.2, yaw=math.pi / 2.0, frame_id='map')

    assert pose.header.frame_id == 'map'
    assert pose.header.stamp.sec == 0
    assert pose.header.stamp.nanosec == 0
    assert pose.pose.pose.position.x == 1.8
    assert pose.pose.pose.position.y == 0.2
    assert math.isclose(pose.pose.pose.orientation.z, math.sin(math.pi / 4.0))
    assert math.isclose(pose.pose.pose.orientation.w, math.cos(math.pi / 4.0))


def test_default_waypoints_after_simple_undock_stay_inside_free_space():
    package_path = Path(__file__).parents[1]
    map_yaml = package_path / 'maps' / 'rectangle_2m_x_1_5m.yaml'
    bounds = load_free_space_bounds(map_yaml)

    waypoints = compute_rectangle_waypoints(
        start_x=1.8,
        start_y=0.3,
        start_yaw=math.pi / 2.0,
        width=0.8,
        length=1.3,
    )

    for waypoint in waypoints:
        assert bounds.contains(waypoint.x, waypoint.y, margin=0.0)
