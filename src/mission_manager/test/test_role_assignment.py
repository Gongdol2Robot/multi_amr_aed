from mission_manager.role_assignment import rank_candidates


def test_lowest_path_cost_is_first():
    result = rank_candidates(
        {
            "robot1": {
                "position": (0.0, 0.0),
                "path_valid": True,
                "path_cost": 7.0,
            },
            "robot2": {
                "position": (5.0, 0.0),
                "path_valid": True,
                "path_cost": 3.0,
            },
        },
        (1.0, 0.0),
    )
    assert result == ["robot2", "robot1"]


def test_invalid_path_is_excluded():
    result = rank_candidates(
        {
            "robot1": {
                "position": (0.0, 0.0),
                "path_valid": False,
                "path_cost": -1.0,
            }
        },
        (1.0, 0.0),
    )
    assert result == []
