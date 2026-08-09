import math

import pytest

from backend.ros.converters import battery_percentage_100


def test_ros_battery_fraction_becomes_display_percent() -> None:
    assert battery_percentage_100(0.91) == pytest.approx(91.0)
    assert battery_percentage_100(1.0) == pytest.approx(100.0)


def test_existing_percent_and_unknown_are_safe() -> None:
    assert battery_percentage_100(73.0) == pytest.approx(73.0)
    assert battery_percentage_100(math.nan) == -1.0
