import math

from multi_robot_emergency.crowd import CrowdStateFilter


def make_filter() -> CrowdStateFilter:
    return CrowdStateFilter(
        level_names=["CLEAR", "BUSY", "CROWDED", "BLOCKED"],
        state_timeout_sec=2.0,
        increase_confirm_sec=0.5,
        decrease_hold_sec=1.5,
    )


def test_no_message_and_stale_message_are_unknown() -> None:
    crowd = make_filter()
    assert crowd.snapshot(0.0).name == "UNKNOWN"
    assert math.isinf(crowd.snapshot(0.0).age_sec)
    assert crowd.update_level("CLEAR", 1.0).name == "CLEAR"
    stale = crowd.snapshot(3.1)
    assert stale.name == "UNKNOWN"
    assert not stale.fresh


def test_vision_stage_increase_requires_confirmation() -> None:
    crowd = make_filter()
    crowd.update_level("CLEAR", 0.0)
    assert crowd.update_level("BUSY", 1.0).name == "CLEAR"
    assert crowd.update_level("busy", 1.49).name == "CLEAR"
    stable = crowd.update_level("BUSY", 1.50)
    assert stable.name == "BUSY"
    assert stable.level == 1


def test_stage_decrease_is_held() -> None:
    crowd = make_filter()
    crowd.update_level("BLOCKED", 0.0)
    crowd.update_level("BLOCKED", 0.5)
    assert crowd.update_level("CROWDED", 1.0).name == "BLOCKED"
    assert crowd.update_level("CROWDED", 2.49).name == "BLOCKED"
    assert crowd.update_level("CROWDED", 2.50).name == "CROWDED"


def test_person_count_is_diagnostic_only() -> None:
    crowd = make_filter()
    crowd.update_level("CLEAR", 0.0)
    crowd.update_person_count(99)
    snapshot = crowd.snapshot(0.1)
    assert snapshot.name == "CLEAR"
    assert snapshot.person_count == 99


def test_unrecognized_team_label_becomes_unknown() -> None:
    crowd = make_filter()
    snapshot = crowd.update_level("TEAM_WILL_RENAME_THIS", 1.0)
    assert snapshot.name == "UNKNOWN"
    assert not snapshot.fresh


def test_numeric_vision_levels_use_amr_stage_names() -> None:
    crowd = make_filter()
    assert crowd.update_level("0", 0.0).name == "CLEAR"
    assert crowd.update_level("2", 1.0).name == "CLEAR"
    snapshot = crowd.update_level("2", 1.5)
    assert snapshot.level == 2
    assert snapshot.name == "CROWDED"


def test_numeric_vision_level_out_of_range_is_unknown() -> None:
    crowd = make_filter()
    snapshot = crowd.update_level("9", 1.0)
    assert snapshot.name == "UNKNOWN"
    assert not snapshot.fresh
