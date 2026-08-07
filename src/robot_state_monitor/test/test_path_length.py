import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import pytest

from robot_state_monitor.monitor_node import (
    battery_percentage_100,
    path_length,
)


def make_path(*points):
    path = Path()
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        path.poses.append(pose)
    return path


def test_path_length_accumulates_every_segment():
    assert path_length(make_path((0.0, 0.0), (3.0, 4.0), (6.0, 4.0))) == 8.0


def test_empty_path_length_is_zero():
    assert path_length(Path()) == 0.0


def test_non_finite_path_is_rejected():
    with pytest.raises(ValueError):
        path_length(make_path((0.0, 0.0), (math.inf, 1.0)))


def test_battery_fraction_is_converted_to_hmi_percent():
    assert battery_percentage_100(0.91) == pytest.approx(91.0)
    assert battery_percentage_100(1.0) == pytest.approx(100.0)


def test_battery_percent_and_unknown_values_are_preserved():
    assert battery_percentage_100(73.0) == pytest.approx(73.0)
    assert battery_percentage_100(float("nan")) == -1.0
