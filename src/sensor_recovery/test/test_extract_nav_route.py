import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[3] / "tools" / "extract_nav_route.py"
SPEC = importlib.util.spec_from_file_location("extract_nav_route", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_extracts_snapshot_while_ignoring_other_lines():
    text = "\n".join(
        [
            "ordinary log",
            'node: NAV_PATH_SNAPSHOT {"start_yaw_deg":10,"points":[[0,0],[1,0]]}',
            "another line",
        ]
    )
    result = MODULE.snapshots_from_log(text)
    assert result == [{"start_yaw_deg": 10, "points": [[0, 0], [1, 0]]}]


def test_extracts_executed_path_separately():
    text = "\n".join(
        [
            'NAV_PATH_SNAPSHOT {"points":[[0,0],[1,0]]}',
            'NAV_EXECUTED_PATH {"points":[[0,0],[0.9,0.1]]}',
        ]
    )
    assert MODULE.snapshots_from_log(text, kind="executed") == [
        {"points": [[0, 0], [0.9, 0.1]]}
    ]
