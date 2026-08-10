#!/usr/bin/env python3
"""Extract a replayable cmd_vel route from a mapnav log snapshot."""

import argparse
import json
from pathlib import Path

import yaml


MARKERS = {
    "planned": "NAV_PATH_SNAPSHOT ",
    "executed": "NAV_EXECUTED_PATH ",
}


def snapshots_from_log(text: str, kind: str = "planned"):
    """Return all valid NAV_PATH_SNAPSHOT dictionaries in log order."""
    marker = MARKERS[kind]
    result = []
    for line in text.splitlines():
        if marker not in line:
            continue
        payload = line.split(marker, 1)[1].strip()
        try:
            result.append(json.loads(payload))
        except json.JSONDecodeError:
            continue
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--snapshot", type=int, default=-1)
    parser.add_argument("--kind", choices=tuple(MARKERS), default="planned")
    parser.add_argument(
        "--start-yaw-deg",
        type=float,
        help=(
            "override start yaw for logs captured before NAV_PATH_SNAPSHOT "
            "used the current AMCL orientation"
        ),
    )
    args = parser.parse_args()
    snapshots = snapshots_from_log(
        args.log.read_text(encoding="utf-8"), kind=args.kind
    )
    if not snapshots:
        raise SystemExit(f"{MARKERS[args.kind].strip()} not found in log")
    selected = snapshots[args.snapshot]
    document = {
        "route": {
            "ready": True,
            "source_log": str(args.log),
            "frame_id": selected.get("frame_id", "map"),
            "start_yaw_deg": (
                selected["start_yaw_deg"]
                if args.start_yaw_deg is None
                else args.start_yaw_deg
            ),
            "goal_yaw_deg": selected.get("goal_yaw_deg"),
            "points": selected["points"],
        }
    }
    args.output.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    print(f"saved {len(selected['points'])} points to {args.output}")


if __name__ == "__main__":
    main()
