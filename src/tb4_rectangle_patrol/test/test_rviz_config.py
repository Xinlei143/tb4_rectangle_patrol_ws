from pathlib import Path


def test_map_displays_use_transient_local_qos():
    rviz_source = (
        Path(__file__).parents[1]
        / 'rviz'
        / 'rectangle_patrol.rviz'
    ).read_text(encoding='utf-8')

    assert rviz_source.count('Durability Policy: Transient Local') >= 3
    assert 'Value: /map' in rviz_source
    assert 'Value: /local_costmap/costmap' in rviz_source
    assert 'Value: /global_costmap/costmap' in rviz_source
