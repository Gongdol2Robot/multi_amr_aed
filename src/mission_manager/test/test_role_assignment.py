import pytest

from mission_manager.role_assignment import select_roles


def test_nearest_robot_gets_aed_role():
    result = select_roles(
        {"robot1": (0.0, 0.0), "robot2": (5.0, 0.0)},
        (1.0, 0.0),
    )
    assert result == ("robot1", "robot2")


def test_requires_two_available_robots():
    with pytest.raises(ValueError):
        select_roles({"robot1": (0.0, 0.0)}, (1.0, 0.0))

