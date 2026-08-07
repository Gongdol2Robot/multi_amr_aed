from pathlib import Path

import yaml


CONFIG_PATH = (
    Path(__file__).resolve().parents[2]
    / 'aed_bringup'
    / 'config'
    / 'nav2_aed.yaml'
)


def _controller_config():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))
    return config['controller_server']['ros__parameters']


def test_progress_checker_counts_rotation_as_progress():
    progress = _controller_config()['progress_checker']
    assert progress['plugin'] == 'nav2_controller::PoseProgressChecker'
    assert 0.05 <= progress['required_movement_radius'] <= 0.15
    assert 0.0 < progress['required_movement_angle'] <= 3.141592653589793


def test_controller_rotates_to_path_before_pure_pursuit():
    follow_path = _controller_config()['FollowPath']
    assert follow_path['plugin'] == (
        'nav2_regulated_pure_pursuit_controller::'
        'RegulatedPurePursuitController'
    )
    assert follow_path['use_rotate_to_heading'] is True
    assert follow_path['allow_reversing'] is False
    assert 0.0 < follow_path['rotate_to_heading_min_angle'] < 1.57


def test_controller_keeps_collision_detection_enabled():
    follow_path = _controller_config()['FollowPath']
    assert follow_path['use_collision_detection'] is True
    assert follow_path['max_allowed_time_to_collision_up_to_carrot'] > 0.0
