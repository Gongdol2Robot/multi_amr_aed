"""Rank two robots and dispatch one or both through Nav2."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import json
import math
import time

from action_msgs.msg import GoalStatus
from aed_interfaces.msg import (
    EmergencyEvent,
    MissionAssignment,
    MissionStatus,
    RobotState,
)
from geometry_msgs.msg import (
    Point,
    PointStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
)
from irobot_create_msgs.msg import DockStatus
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32, String, UInt32
from visualization_msgs.msg import Marker, MarkerArray

from .assignment import (
    crowd_delay_seconds,
    dispatch_candidates,
    path_length,
    path_length_in_polygon,
    path_motion_cost,
    patient_standoff,
    point_to_polygon_distance,
    proximity_retreat_candidate,
    should_switch_for_live_eta,
)
from .crowd import CrowdSnapshot, CrowdStateFilter


@dataclass
class RobotObservation:
    """Most recently received map pose for one robot."""

    pose: PoseStamped
    received_at: float


class EmergencyMissionManager(Node):
    """Compare both Nav2 plans and manage deadline-aware dispatch."""

    def __init__(self) -> None:
        """Create robot inputs, mission outputs, and Nav2 action clients."""
        super().__init__("emergency_mission_manager")
        self.declare_parameter("robot_ids", ["robot1", "robot2"])
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("request_topic", "/emergency/request")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter(
            "emergency_event_topics",
            [
                "/aed/emergency_event",
                "/camera_open/vision/emergency_event",
                "/camera_alley/vision/emergency_event",
            ],
        )
        self.declare_parameter("pose_timeout_sec", 15.0)
        self.declare_parameter("allow_stale_pose", True)
        self.declare_parameter("use_planner_start", True)
        self.declare_parameter("docked_start_offset_m", 0.35)
        self.declare_parameter("planning_timeout_sec", 30.0)
        self.declare_parameter("dispatch_retry_timeout_sec", 15.0)
        self.declare_parameter("assignment_ack_timeout_sec", 3.0)
        self.declare_parameter("live_replan_enabled", True)
        self.declare_parameter("live_replan_interval_sec", 3.0)
        self.declare_parameter("live_replan_timeout_sec", 4.0)
        self.declare_parameter("live_replan_min_eta_gain_sec", 2.0)
        self.declare_parameter("live_replan_switch_ratio", 0.85)
        self.declare_parameter("dual_dispatch_enabled", True)
        self.declare_parameter("target_arrival_time_sec", 30.0)
        self.declare_parameter("dual_dispatch_trigger_ratio", 0.85)
        self.declare_parameter("patient_standoff_enabled", True)
        # OAK-D가 환자와 helper를 한 화면에 유지할 수 있는 최소 운용 거리.
        self.declare_parameter("patient_standoff_distance_m", 0.60)
        self.declare_parameter("return_after_helper_enabled", True)
        self.declare_parameter("dual_robot_proximity_threshold_m", 0.40)
        self.declare_parameter("dual_robot_proximity_confirm_sec", 0.50)
        self.declare_parameter("dual_robot_proximity_grace_sec", 2.0)
        self.declare_parameter("nominal_linear_speed_mps", 0.20)
        self.declare_parameter("nominal_angular_speed_radps", 0.70)
        self.declare_parameter("slowdown_turn_threshold_deg", 45.0)
        self.declare_parameter("slowdown_penalty_sec", 4.0)
        self.declare_parameter("path_simplification_tolerance_m", 0.10)
        self.declare_parameter(
            "crowd_person_count_topic",
            "/camera_alley/vision/person_count",
        )
        self.declare_parameter(
            "crowd_level_topic", "/camera_alley/vision/crowd_level"
        )
        self.declare_parameter(
            "crowd_level_names", ["CLEAR", "BUSY", "CROWDED", "BLOCKED"]
        )
        self.declare_parameter("crowd_state_timeout_sec", 2.0)
        self.declare_parameter("crowd_increase_confirm_sec", 0.5)
        self.declare_parameter("crowd_decrease_hold_sec", 1.5)
        self.declare_parameter(
            "crowd_level_speeds_mps", [0.20, 0.15, 0.10, 0.05]
        )
        self.declare_parameter("crowd_blocking_level", 3)
        self.declare_parameter("crowd_keepout_margin_m", 0.22)
        self.declare_parameter("crowd_keepout_settle_sec", 1.2)
        self.declare_parameter(
            "crowd_keepout_mask_topic", "/emergency/crowd_keepout_mask"
        )
        self.declare_parameter(
            "crowd_filter_info_topic", "/emergency/crowd_filter_info"
        )
        self.declare_parameter(
            "crowd_zone_polygon",
            [
                -1.6129,
                1.8464,
                -1.9610,
                2.2231,
                -2.9529,
                2.0745,
                -2.8543,
                1.4191,
            ],
        )
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("dispatch_enabled", False)
        self.declare_parameter("automatic_request", False)
        self.declare_parameter("automatic_request_delay_sec", 3.0)
        self.declare_parameter("initial_target_x", 1.2)
        self.declare_parameter("initial_target_y", 2.4)
        self.declare_parameter("initial_target_yaw", 0.0)

        self.robot_ids = [
            str(value) for value in self.get_parameter("robot_ids").value
        ]
        if len(self.robot_ids) != 2 or len(set(self.robot_ids)) != 2:
            raise ValueError("robot_ids must contain exactly two unique IDs")
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.pose_timeout = float(
            self.get_parameter("pose_timeout_sec").value
        )
        self.allow_stale_pose = bool(
            self.get_parameter("allow_stale_pose").value
        )
        self.use_planner_start = bool(
            self.get_parameter("use_planner_start").value
        )
        self.docked_start_offset = float(
            self.get_parameter("docked_start_offset_m").value
        )
        self.planning_timeout = float(
            self.get_parameter("planning_timeout_sec").value
        )
        self.dispatch_retry_timeout = float(
            self.get_parameter("dispatch_retry_timeout_sec").value
        )
        self.assignment_ack_timeout = float(
            self.get_parameter("assignment_ack_timeout_sec").value
        )
        self.live_replan_enabled = bool(
            self.get_parameter("live_replan_enabled").value
        )
        self.live_replan_interval = float(
            self.get_parameter("live_replan_interval_sec").value
        )
        self.live_replan_timeout = float(
            self.get_parameter("live_replan_timeout_sec").value
        )
        self.live_replan_min_eta_gain = float(
            self.get_parameter("live_replan_min_eta_gain_sec").value
        )
        self.live_replan_switch_ratio = float(
            self.get_parameter("live_replan_switch_ratio").value
        )
        self.dual_dispatch_enabled = bool(
            self.get_parameter("dual_dispatch_enabled").value
        )
        self.target_arrival_time = float(
            self.get_parameter("target_arrival_time_sec").value
        )
        self.dual_dispatch_trigger_ratio = float(
            self.get_parameter("dual_dispatch_trigger_ratio").value
        )
        self.patient_standoff_enabled = bool(
            self.get_parameter("patient_standoff_enabled").value
        )
        self.patient_standoff_distance = float(
            self.get_parameter("patient_standoff_distance_m").value
        )
        self.return_after_helper = bool(
            self.get_parameter("return_after_helper_enabled").value
        )
        self.dual_robot_proximity_threshold = float(
            self.get_parameter("dual_robot_proximity_threshold_m").value
        )
        self.dual_robot_proximity_confirm = float(
            self.get_parameter("dual_robot_proximity_confirm_sec").value
        )
        self.dual_robot_proximity_grace = float(
            self.get_parameter("dual_robot_proximity_grace_sec").value
        )
        self.nominal_linear_speed = float(
            self.get_parameter("nominal_linear_speed_mps").value
        )
        self.nominal_angular_speed = float(
            self.get_parameter("nominal_angular_speed_radps").value
        )
        self.slowdown_turn_threshold = math.radians(
            float(self.get_parameter("slowdown_turn_threshold_deg").value)
        )
        self.slowdown_penalty = float(
            self.get_parameter("slowdown_penalty_sec").value
        )
        self.path_simplification_tolerance = float(
            self.get_parameter("path_simplification_tolerance_m").value
        )
        self.crowd_level_speeds = [
            float(value)
            for value in self.get_parameter("crowd_level_speeds_mps").value
        ]
        self.crowd_blocking_level = int(
            self.get_parameter("crowd_blocking_level").value
        )
        self.crowd_keepout_margin = float(
            self.get_parameter("crowd_keepout_margin_m").value
        )
        self.crowd_keepout_settle = float(
            self.get_parameter("crowd_keepout_settle_sec").value
        )
        self.crowd_keepout_mask_topic = str(
            self.get_parameter("crowd_keepout_mask_topic").value
        )
        self.crowd_filter_info_topic = str(
            self.get_parameter("crowd_filter_info_topic").value
        )
        polygon_values = [
            float(value)
            for value in self.get_parameter("crowd_zone_polygon").value
        ]
        if len(polygon_values) < 6 or len(polygon_values) % 2:
            raise ValueError(
                "crowd_zone_polygon must contain at least three x,y pairs"
            )
        self.crowd_zone_polygon = list(
            zip(polygon_values[::2], polygon_values[1::2])
        )
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.dispatch_enabled = bool(
            self.get_parameter("dispatch_enabled").value
        )
        if self.pose_timeout <= 0.0:
            raise ValueError("pose_timeout_sec must be positive")
        if self.planning_timeout <= 0.0:
            raise ValueError("planning_timeout_sec must be positive")
        if self.dispatch_retry_timeout <= 0.0:
            raise ValueError("dispatch_retry_timeout_sec must be positive")
        if self.assignment_ack_timeout <= 0.0:
            raise ValueError("assignment_ack_timeout_sec must be positive")
        for name, duration in (
            ("live_replan_interval_sec", self.live_replan_interval),
            ("live_replan_timeout_sec", self.live_replan_timeout),
        ):
            if not math.isfinite(duration) or duration <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if (
            not math.isfinite(self.live_replan_min_eta_gain)
            or self.live_replan_min_eta_gain < 0.0
        ):
            raise ValueError(
                "live_replan_min_eta_gain_sec must be finite and non-negative"
            )
        if (
            not math.isfinite(self.live_replan_switch_ratio)
            or not 0.0 < self.live_replan_switch_ratio <= 1.0
        ):
            raise ValueError("live_replan_switch_ratio must be in (0, 1]")
        if (
            not math.isfinite(self.target_arrival_time)
            or self.target_arrival_time <= 0.0
        ):
            raise ValueError("target_arrival_time_sec must be positive")
        if (
            not math.isfinite(self.dual_dispatch_trigger_ratio)
            or self.dual_dispatch_trigger_ratio <= 0.0
            or self.dual_dispatch_trigger_ratio > 1.0
        ):
            raise ValueError(
                "dual_dispatch_trigger_ratio must be in the (0, 1] interval"
            )
        if (
            not math.isfinite(self.patient_standoff_distance)
            or self.patient_standoff_distance <= 0.0
        ):
            raise ValueError("patient_standoff_distance_m must be positive")
        if (
            not math.isfinite(self.dual_robot_proximity_threshold)
            or self.dual_robot_proximity_threshold <= 0.0
        ):
            raise ValueError(
                "dual_robot_proximity_threshold_m must be positive"
            )
        for name, duration in (
            (
                "dual_robot_proximity_confirm_sec",
                self.dual_robot_proximity_confirm,
            ),
            (
                "dual_robot_proximity_grace_sec",
                self.dual_robot_proximity_grace,
            ),
        ):
            if not math.isfinite(duration) or duration < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.nominal_linear_speed <= 0.0:
            raise ValueError("nominal_linear_speed_mps must be positive")
        if self.nominal_angular_speed <= 0.0:
            raise ValueError("nominal_angular_speed_radps must be positive")
        if self.slowdown_turn_threshold < 0.0:
            raise ValueError(
                "slowdown_turn_threshold_deg must be non-negative"
            )
        if self.slowdown_penalty < 0.0:
            raise ValueError("slowdown_penalty_sec must be non-negative")
        if (
            not math.isfinite(self.path_simplification_tolerance)
            or self.path_simplification_tolerance < 0.0
        ):
            raise ValueError(
                "path_simplification_tolerance_m must be finite and "
                "non-negative"
            )
        crowd_level_names = [
            str(value)
            for value in self.get_parameter("crowd_level_names").value
        ]
        if len(self.crowd_level_speeds) != len(crowd_level_names):
            raise ValueError(
                "crowd_level_speeds_mps must match crowd_level_names"
            )
        if not all(
            0.0 < speed <= self.nominal_linear_speed
            for speed in self.crowd_level_speeds
        ):
            raise ValueError(
                "crowd speeds must be positive and no faster than normal"
            )
        if not 1 <= self.crowd_blocking_level < len(crowd_level_names):
            raise ValueError("crowd_blocking_level is outside stage range")
        if self.crowd_keepout_margin < 0.0:
            raise ValueError("crowd_keepout_margin_m must be non-negative")
        if self.crowd_keepout_settle < 0.0:
            raise ValueError("crowd_keepout_settle_sec must be non-negative")
        self.crowd_filter = CrowdStateFilter(
            level_names=crowd_level_names,
            state_timeout_sec=float(
                self.get_parameter("crowd_state_timeout_sec").value
            ),
            increase_confirm_sec=float(
                self.get_parameter("crowd_increase_confirm_sec").value
            ),
            decrease_hold_sec=float(
                self.get_parameter("crowd_decrease_hold_sec").value
            ),
        )
        self.raw_crowd_level = "UNKNOWN"
        if self.docked_start_offset < 0.0:
            raise ValueError("docked_start_offset_m must be non-negative")

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            String, "/emergency/status", latched_qos
        )
        self.selected_publisher = self.create_publisher(
            String, "/emergency/selected_robot", latched_qos
        )
        self.dispatched_publisher = self.create_publisher(
            String, "/emergency/dispatched_robots", latched_qos
        )
        self.target_marker_publisher = self.create_publisher(
            Marker, "/emergency/target_marker", latched_qos
        )
        self.robot_marker_publisher = self.create_publisher(
            MarkerArray, "/emergency/robot_markers", latched_qos
        )
        self.path_publishers = {
            robot_id: self.create_publisher(
                Path,
                f"/emergency/candidate_path/{robot_id}",
                latched_qos,
            )
            for robot_id in self.robot_ids
        }
        self.distance_publishers = {
            robot_id: self.create_publisher(
                Float32,
                f"/emergency/path_distance/{robot_id}",
                latched_qos,
            )
            for robot_id in self.robot_ids
        }
        self.predicted_eta_publishers = {
            robot_id: self.create_publisher(
                Float32,
                f"/emergency/eta/predicted/{robot_id}",
                latched_qos,
            )
            for robot_id in self.robot_ids
        }
        self.actual_eta_publishers = {
            robot_id: self.create_publisher(
                Float32,
                f"/emergency/eta/actual/{robot_id}",
                latched_qos,
            )
            for robot_id in self.robot_ids
        }
        eta_result_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.eta_result_publisher = self.create_publisher(
            String, "/emergency/eta/result", eta_result_qos
        )
        self.crowd_state_publisher = self.create_publisher(
            String, "/emergency/crowd/state", latched_qos
        )
        self.crowd_marker_publisher = self.create_publisher(
            MarkerArray, "/emergency/crowd_markers", latched_qos
        )
        self.keepout_mask_publisher = self.create_publisher(
            OccupancyGrid, self.crowd_keepout_mask_topic, latched_qos
        )
        self.filter_info_publisher = self.create_publisher(
            CostmapFilterInfo, self.crowd_filter_info_topic, latched_qos
        )
        self.crowded_distance_publishers = {
            robot_id: self.create_publisher(
                Float32,
                f"/emergency/crowded_path_distance/{robot_id}",
                latched_qos,
            )
            for robot_id in self.robot_ids
        }
        self.crowd_delay_publishers = {
            robot_id: self.create_publisher(
                Float32,
                f"/emergency/crowd_delay/{robot_id}",
                latched_qos,
            )
            for robot_id in self.robot_ids
        }

        self.latest_map: OccupancyGrid | None = None
        self.keepout_active: bool | None = None
        self.observations: dict[str, RobotObservation] = {}
        self.dock_states: dict[str, bool] = {}
        self.assignment_publishers = {
            robot_id: self.create_publisher(
                MissionAssignment,
                f"/{robot_id}/mission_assignment",
                10,
            )
            for robot_id in self.robot_ids
        }
        self.mission_status_subscription = self.create_subscription(
            MissionStatus,
            "/aed/mission_status",
            self._on_mission_status,
            20,
        )
        self.crowd_person_count_subscription = self.create_subscription(
            UInt32,
            str(self.get_parameter("crowd_person_count_topic").value),
            self._on_crowd_person_count,
            10,
        )
        self.crowd_level_subscription = self.create_subscription(
            String,
            str(self.get_parameter("crowd_level_topic").value),
            self._on_raw_crowd_level,
            10,
        )
        # Robot1이 꺼진 Robot2 단독 시험에서도 keepout mask를 만들 수
        # 있도록 두 map_server 중 먼저 보이는 공용 지도를 사용한다.
        self.map_subscriptions = [
            self.create_subscription(
                OccupancyGrid,
                f"/{robot_id}/map",
                self._on_map,
                latched_qos,
            )
            for robot_id in self.robot_ids
        ]
        self.planner_clients = {
            robot_id: ActionClient(
                self,
                ComputePathToPose,
                f"/{robot_id}/compute_path_to_pose",
            )
            for robot_id in self.robot_ids
        }
        self.pose_subscriptions = [
            self.create_subscription(
                PoseWithCovarianceStamped,
                f"/{robot_id}/amcl_pose",
                lambda message, rid=robot_id: self._on_pose(rid, message),
                # AMCL publishes a transient-local pose. Matching that QoS
                # gives a newly started central manager the last known pose
                # even while the robot is stationary.
                latched_qos,
            )
            for robot_id in self.robot_ids
        ]
        self.dock_subscriptions = [
            self.create_subscription(
                DockStatus,
                f"/{robot_id}/dock_status",
                lambda message, rid=robot_id: self._on_dock_status(
                    rid, message
                ),
                latched_qos,
            )
            for robot_id in self.robot_ids
        ]
        self.request_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("request_topic").value),
            self._on_request,
            10,
        )
        event_topics = [
            str(topic)
            for topic in self.get_parameter("emergency_event_topics").value
        ]
        self.processed_event_ids: set[str] = set()
        self.emergency_event_subscriptions = [
            self.create_subscription(
                EmergencyEvent,
                topic,
                self._on_emergency_event,
                10,
            )
            for topic in dict.fromkeys(event_topics)
        ]
        global_click_topic = str(
            self.get_parameter("clicked_point_topic").value
        )
        click_topics = [global_click_topic]
        click_topics.extend(
            f"/{robot_id}/clicked_point" for robot_id in self.robot_ids
        )
        self.clicked_point_subscriptions = [
            self.create_subscription(
                PointStamped,
                topic,
                self._on_clicked_point,
                10,
            )
            for topic in dict.fromkeys(click_topics)
        ]
        self.marker_timer = self.create_timer(0.5, self._publish_robot_markers)

        self.request_serial = 0
        self.navigation_active = False
        self.planning_active = False
        self.pending_plans: set[str] = set()
        self.plan_results: dict[
            str,
            tuple[Path, float, float, float, int, float, float, float, str],
        ] = {}
        self.plan_failures: dict[str, str] = {}
        self.planning_target: PoseStamped | None = None
        self.planning_targets: dict[str, PoseStamped] = {}
        self.planning_timer = None
        self.selected_robot = ""
        self.ranked_candidates: list[str] = []
        self.excluded_robots: set[str] = set()
        self.assignment_version = 0
        self.assignment_versions: dict[str, int] = {}
        self.assignment_ack_timers: dict[str, object] = {}
        self.active_request_id = ""
        self.state = "IDLE"
        self._last_feedback_log = 0.0
        self.navigation_started_at: dict[str, float] = {}
        self.navigation_predicted_eta: dict[str, float] = {}
        self.dispatch_start_poses: dict[str, PoseStamped] = {}
        self.dispatched_robots: set[str] = set()
        self.terminal_robots: set[str] = set()
        self.arrived_robots: set[str] = set()
        self.failed_robots: set[str] = set()
        self.return_failed_robots: set[str] = set()
        self.returning_robots: set[str] = set()
        self.awaiting_helper_robots: set[str] = set()
        self.dual_dispatch_active = False
        self.dual_dispatch_started_at: float | None = None
        self.proximity_close_since: float | None = None
        self.proximity_return_triggered = False
        self.live_replan_active = False
        self.live_replan_serial = 0
        self.live_replan_started_at = 0.0
        self.live_replan_next_at = time.monotonic() + self.live_replan_interval
        self.live_replan_pending: set[str] = set()
        self.live_replan_results: dict[
            str,
            tuple[Path, float, float, float, int, float, float, float, str],
        ] = {}
        self.live_replan_failures: dict[str, str] = {}
        self.live_replan_crowd_snapshot = CrowdSnapshot(
            -1, "UNKNOWN", 0, False, math.inf
        )
        self.live_reassignment_done = False
        self.planning_crowd_snapshot = CrowdSnapshot(
            -1, "UNKNOWN", 0, False, math.inf
        )
        self._automatic_request_timer = None
        self.proximity_timer = self.create_timer(
            0.1, self._monitor_dual_robot_proximity
        )
        self.live_replan_timer = self.create_timer(
            0.5, self._monitor_live_replan
        )
        if bool(self.get_parameter("automatic_request").value):
            delay = float(
                self.get_parameter("automatic_request_delay_sec").value
            )
            self._automatic_request_timer = self.create_timer(
                max(0.1, delay), self._publish_automatic_request
            )

        self._publish_status("IDLE", "waiting for an emergency request")
        self._publish_crowd_state(self.planning_crowd_snapshot)
        self.get_logger().info(
            "Emergency request: ros2 topic pub --once /emergency/request "
            "geometry_msgs/msg/PoseStamped "
            "'{header: {frame_id: map}, pose: {position: {x: 1.2, "
            "y: 2.4}, orientation: {w: 1.0}}}'"
        )
        self.get_logger().info(
            f"Dispatch enabled: {self.dispatch_enabled}; "
            f"robots={self.robot_ids}"
        )
        trigger_eta = (
            self.target_arrival_time * self.dual_dispatch_trigger_ratio
        )
        self.get_logger().info(
            "Dual dispatch: "
            f"enabled={self.dual_dispatch_enabled}, "
            f"target={self.target_arrival_time:.1f}s, "
            f"trigger={trigger_eta:.1f}s"
        )
        self.get_logger().info(
            "Patient standoff: "
            f"enabled={self.patient_standoff_enabled}, "
            f"distance={self.patient_standoff_distance:.2f}m"
        )
        self.get_logger().info(
            "Dual proximity return: "
            f"threshold={self.dual_robot_proximity_threshold:.2f}m, "
            f"confirm={self.dual_robot_proximity_confirm:.2f}s, "
            f"grace={self.dual_robot_proximity_grace:.2f}s"
        )
        self.get_logger().info(
            "Live ETA reassignment: "
            f"enabled={self.live_replan_enabled}, "
            f"interval={self.live_replan_interval:.1f}s, "
            f"gain>={self.live_replan_min_eta_gain:.1f}s, "
            f"ratio<={self.live_replan_switch_ratio:.2f}"
        )
        self.get_logger().info(
            "RViz Publish Point topics: " + ", ".join(click_topics)
        )
        self.get_logger().info(
            "YOLO emergency event topics: " + ", ".join(event_topics)
        )
        self.get_logger().info(
            "Crowd input: "
            f"{self.get_parameter('crowd_level_topic').value}; "
            "person_count is diagnostics only"
        )

    def _on_pose(
        self, robot_id: str, message: PoseWithCovarianceStamped
    ) -> None:
        if (
            message.header.frame_id
            and message.header.frame_id != self.map_frame
        ):
            self.get_logger().warning(
                f"Ignoring {robot_id} pose in frame "
                f"{message.header.frame_id!r}; expected {self.map_frame!r}"
            )
            return
        pose = PoseStamped()
        pose.header = deepcopy(message.header)
        pose.header.frame_id = self.map_frame
        pose.pose = deepcopy(message.pose.pose)
        self.observations[robot_id] = RobotObservation(
            pose=pose,
            received_at=time.monotonic(),
        )

    def _on_dock_status(self, robot_id: str, message: DockStatus) -> None:
        self.dock_states[robot_id] = bool(message.is_docked)

    def _on_crowd_person_count(self, message: UInt32) -> None:
        """Keep the count for diagnostics; vision owns classification."""
        self.crowd_filter.update_person_count(int(message.data))

    def _on_raw_crowd_level(self, message: String) -> None:
        """Consume the final crowd decision made by the vision node."""
        previous = self.crowd_filter.snapshot(time.monotonic())
        self.raw_crowd_level = message.data.strip() or "UNKNOWN"
        snapshot = self.crowd_filter.update_level(
            self.raw_crowd_level, time.monotonic()
        )
        self._publish_crowd_state(snapshot)
        if (
            self.navigation_active
            and snapshot.fresh
            and snapshot.level != previous.level
        ):
            # Give both Nav2 costmaps time to consume a changed keepout mask,
            # then compare fresh paths instead of waiting for the periodic run.
            self.live_replan_next_at = time.monotonic() + (
                self.crowd_keepout_settle
                if snapshot.level >= self.crowd_blocking_level
                or previous.level >= self.crowd_blocking_level
                else 0.0
            )
            self.get_logger().info(
                "Crowd stage changed during navigation: "
                f"{previous.name}->{snapshot.name}; scheduling live ETA check"
            )

    def _on_map(self, message: OccupancyGrid) -> None:
        """Keep one shared map template for the dynamic keepout mask."""
        self.latest_map = deepcopy(message)
        self.keepout_active = None
        self._publish_keepout_mask(
            self.crowd_filter.snapshot(time.monotonic()), force=True
        )

    def _on_request(
        self, message: PoseStamped, request_id: str | None = None
    ) -> None:
        request = deepcopy(message)
        if not request.header.frame_id:
            request.header.frame_id = self.map_frame
        if request.header.frame_id != self.map_frame:
            self._publish_status(
                "FAILED",
                f"request frame must be {self.map_frame}, got "
                f"{request.header.frame_id}",
            )
            return
        if not self._finite_pose(request):
            self._publish_status(
                "FAILED", "request pose contains non-finite data"
            )
            return
        if self.planning_active or self.navigation_active:
            self.get_logger().warning(
                "Ignoring a new emergency request while a mission is active"
            )
            return

        self.request_serial += 1
        self.assignment_version = 0
        self.assignment_versions.clear()
        self.ranked_candidates.clear()
        self.excluded_robots.clear()
        self.navigation_started_at.clear()
        self.navigation_predicted_eta.clear()
        self.dispatch_start_poses.clear()
        self.planning_targets.clear()
        self.dispatched_robots.clear()
        self.terminal_robots.clear()
        self.arrived_robots.clear()
        self.failed_robots.clear()
        self.return_failed_robots.clear()
        self.returning_robots.clear()
        self.awaiting_helper_robots.clear()
        self.dual_dispatch_active = False
        self.dual_dispatch_started_at = None
        self.proximity_close_since = None
        self.proximity_return_triggered = False
        self.live_replan_active = False
        self.live_replan_pending.clear()
        self.live_replan_results.clear()
        self.live_replan_failures.clear()
        self.live_reassignment_done = False
        self.live_replan_next_at = time.monotonic() + self.live_replan_interval
        self._publish_dispatched_robots()
        request.header.stamp = self.get_clock().now().to_msg()
        self.active_request_id = (
            request_id.strip()
            if request_id is not None and request_id.strip()
            else f"emergency-{self.request_serial:03d}"
        )
        self._publish_status("EMERGENCY_RECEIVED", self.active_request_id)
        self._publish_target_marker(request)
        crowd = self.crowd_filter.snapshot(time.monotonic())
        if crowd.fresh and crowd.level >= self.crowd_blocking_level:
            if self.latest_map is None:
                self._publish_status(
                    "FAILED", "BLOCKED crowd stage but keepout map unavailable"
                )
                return
            self._publish_keepout_mask(crowd, force=True)
            if self.crowd_keepout_settle > 0.0:
                self.planning_active = True
                self.planning_target = deepcopy(request)
                self._publish_status(
                    "CALCULATING",
                    f"waiting {self.crowd_keepout_settle:.1f}s for Nav2 "
                    "keepout update",
                )
                serial = self.request_serial
                target = deepcopy(request)
                self.planning_timer = self.create_timer(
                    self.crowd_keepout_settle,
                    lambda: self._begin_keepout_planning(serial, target),
                )
                return
        self._calculate_and_assign(request)

    def _on_emergency_event(self, message: EmergencyEvent) -> None:
        """Start one mission on a YOLO confirmation edge only."""
        if message.status != EmergencyEvent.CONFIRMED:
            return
        event_id = message.event_id.strip()
        if event_id and event_id in self.processed_event_ids:
            return
        if event_id:
            self.processed_event_ids.add(event_id)

        request = PoseStamped()
        request.header = deepcopy(message.location.header)
        request.pose.position = deepcopy(message.location.point)
        request.pose.orientation.w = 1.0
        self.get_logger().info(
            "YOLO emergency confirmed: "
            f"event={event_id or '<empty>'}, camera={message.camera_id}, "
            f"confidence={message.confidence:.3f}, "
            f"x={request.pose.position.x:.3f}, "
            f"y={request.pose.position.y:.3f}"
        )
        self._on_request(request, request_id=event_id or None)

    def _begin_keepout_planning(
        self, serial: int, target: PoseStamped
    ) -> None:
        """Plan after both Nav2 costmaps have consumed the keepout mask."""
        if serial != self.request_serial or not self.planning_active:
            return
        if self.planning_timer is not None:
            timer = self.planning_timer
            self.planning_timer = None
            timer.cancel()
            self.destroy_timer(timer)
        self.planning_active = False
        self._calculate_and_assign(target)

    def _on_clicked_point(self, message: PointStamped) -> None:
        """Convert an RViz Publish Point click into a planning request."""
        request = PoseStamped()
        request.header = deepcopy(message.header)
        request.pose.position = deepcopy(message.point)
        request.pose.orientation.w = 1.0
        self.get_logger().info(
            "RViz point received: "
            f"x={message.point.x:.3f}, y={message.point.y:.3f}"
        )
        self._on_request(request)

    def _make_standoff_target(
        self, robot_id: str, patient: PoseStamped
    ) -> PoseStamped:
        """Place a robot on the standoff circle facing the patient."""
        if not self.patient_standoff_enabled:
            return deepcopy(patient)
        observation = self.observations.get(robot_id)
        if observation is None:
            raise ValueError("no AMCL pose for patient standoff")
        robot_position = observation.pose.pose.position
        patient_position = patient.pose.position
        stop_x, stop_y, facing_yaw = patient_standoff(
            (patient_position.x, patient_position.y),
            (robot_position.x, robot_position.y),
            self.patient_standoff_distance,
            fallback_robot_yaw=self._quaternion_yaw(observation.pose),
        )
        target = deepcopy(patient)
        target.pose.position.x = stop_x
        target.pose.position.y = stop_y
        target.pose.position.z = 0.0
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = math.sin(facing_yaw * 0.5)
        target.pose.orientation.w = math.cos(facing_yaw * 0.5)
        return target

    def _calculate_and_assign(self, target: PoseStamped) -> None:
        self._publish_status("CALCULATING", "requesting both Nav2 paths")
        now = time.monotonic()
        self.planning_crowd_snapshot = self.crowd_filter.snapshot(now)
        self._publish_crowd_state(self.planning_crowd_snapshot)
        crowd = self.planning_crowd_snapshot
        self.get_logger().info(
            f"Crowd snapshot: level={crowd.name}, "
            f"people={crowd.person_count}, fresh={crowd.fresh}, "
            f"raw={self.raw_crowd_level}"
        )
        self.planning_active = True
        self.planning_target = deepcopy(target)
        self.pending_plans.clear()
        self.plan_results.clear()
        self.plan_failures.clear()
        self.planning_targets.clear()
        for robot_id in self.robot_ids:
            try:
                self.planning_targets[robot_id] = (
                    self._make_standoff_target(robot_id, target)
                )
            except ValueError as error:
                self.plan_failures[robot_id] = str(error)
        for robot_id in self.robot_ids:
            self._publish_distance(robot_id, math.nan)
            self._publish_predicted_eta(robot_id, math.nan)
            self._publish_actual_eta(robot_id, math.nan)
            self._publish_crowd_metrics(robot_id, math.nan, math.nan)

        for robot_id in self.robot_ids:
            if robot_id in self.plan_failures:
                continue
            client = self.planner_clients[robot_id]
            if not client.wait_for_server(timeout_sec=1.0):
                self.plan_failures[robot_id] = (
                    f"/{robot_id}/compute_path_to_pose unavailable"
                )
                continue

            goal = ComputePathToPose.Goal()
            goal.goal = deepcopy(self.planning_targets[robot_id])
            stamp = self.get_clock().now().to_msg()
            goal.goal.header.stamp = stamp
            goal.planner_id = self.planner_id
            docked = self.dock_states.get(robot_id, False)
            goal.use_start = docked or not self.use_planner_start
            if goal.use_start:
                observation = self.observations.get(robot_id)
                if observation is None:
                    self.plan_failures[robot_id] = "no AMCL pose"
                    continue
                age = now - observation.received_at
                if age > self.pose_timeout:
                    if not self.allow_stale_pose:
                        self.plan_failures[robot_id] = (
                            f"stale pose {age:.1f}s"
                        )
                        continue
                    self.get_logger().warning(
                        f"{robot_id}: using last AMCL pose "
                        f"({age:.1f}s old)"
                    )
                goal.start = deepcopy(observation.pose)
                if docked:
                    goal.start = self._project_undocked_start(goal.start)
                    self.get_logger().info(
                        f"{robot_id}: docked; plan from predicted "
                        f"undocked pose ({goal.start.pose.position.x:.2f}, "
                        f"{goal.start.pose.position.y:.2f})"
                    )
                goal.start.header.stamp = stamp
            self.pending_plans.add(robot_id)
            future = client.send_goal_async(goal)
            future.add_done_callback(
                lambda response, rid=robot_id, serial=self.request_serial:
                self._on_plan_response(rid, serial, response)
            )

        for robot_id in self.plan_failures:
            self.path_publishers[robot_id].publish(Path())

        if not self.pending_plans:
            self._finish_planning(self.request_serial)
            return
        self.planning_timer = self.create_timer(
            self.planning_timeout,
            lambda serial=self.request_serial:
            self._on_planning_timeout(serial),
        )

    def _on_plan_response(self, robot_id: str, serial: int, future) -> None:
        if serial != self.request_serial or robot_id not in self.pending_plans:
            return
        try:
            handle = future.result()
        except Exception as error:
            self._record_plan_failure(
                robot_id, serial, f"request error: {error}"
            )
            return
        if not handle.accepted:
            self._record_plan_failure(
                robot_id, serial, "planner rejected goal"
            )
            return
        handle.get_result_async().add_done_callback(
            lambda result, rid=robot_id: self._on_plan_result(
                rid, serial, result
            )
        )

    def _on_plan_result(self, robot_id: str, serial: int, future) -> None:
        if serial != self.request_serial or robot_id not in self.pending_plans:
            return
        try:
            wrapped_result = future.result()
            path = wrapped_result.result.path
            status = int(wrapped_result.status)
        except Exception as error:
            self._record_plan_failure(
                robot_id, serial, f"result error: {error}"
            )
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._record_plan_failure(
                robot_id, serial, f"planner result status={status}"
            )
            return
        if not path.poses:
            self._record_plan_failure(
                robot_id, serial, "planner returned empty path"
            )
            return

        try:
            points = [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in path.poses
            ]
            distance = path_length(points)
            observation = self.observations.get(robot_id)
            initial_yaw = (
                self._quaternion_yaw(observation.pose)
                if observation is not None
                else None
            )
            final_yaw = (
                self._quaternion_yaw(self.planning_targets[robot_id])
                if robot_id in self.planning_targets
                else None
            )
            base_eta, turn_angle, slowdown_count = path_motion_cost(
                points,
                linear_speed=self.nominal_linear_speed,
                angular_speed=self.nominal_angular_speed,
                slowdown_turn_threshold=self.slowdown_turn_threshold,
                slowdown_penalty=self.slowdown_penalty,
                simplification_tolerance=(
                    self.path_simplification_tolerance
                ),
                initial_yaw=initial_yaw,
                final_yaw=final_yaw,
            )
            crowded_distance = path_length_in_polygon(
                points, self.crowd_zone_polygon
            )
            crowd = self.planning_crowd_snapshot
            if (
                crowd.fresh
                and crowd.level >= self.crowd_blocking_level
                and crowded_distance > 1e-6
            ):
                self._record_plan_failure(
                    robot_id,
                    serial,
                    f"crowd stage {crowd.name}: blocked zone intersects "
                    f"{crowded_distance:.2f}m of path",
                )
                return
            crowd_speed = self._crowd_speed(crowd)
            crowd_delay = (
                crowd_delay_seconds(
                    crowded_distance,
                    normal_speed=self.nominal_linear_speed,
                    crowded_speed=crowd_speed,
                )
                if crowd.fresh and crowd.level > 0
                else 0.0
            )
            eta = base_eta + crowd_delay
        except ValueError as error:
            self._record_plan_failure(robot_id, serial, str(error))
            return

        self.plan_results[robot_id] = (
            path,
            distance,
            eta,
            turn_angle,
            slowdown_count,
            base_eta,
            crowded_distance,
            crowd_delay,
            crowd.name,
        )
        self.path_publishers[robot_id].publish(path)
        self._publish_distance(robot_id, distance)
        self._publish_predicted_eta(robot_id, eta)
        self._publish_crowd_metrics(
            robot_id, crowded_distance, crowd_delay
        )
        self.pending_plans.discard(robot_id)
        stop = self.planning_targets[robot_id].pose.position
        self.get_logger().info(
            f"Candidate {robot_id}: distance={distance:.2f}m, "
            f"turn={math.degrees(turn_angle):.1f}deg, "
            f"slowdowns={slowdown_count}, base_eta={base_eta:.2f}s, "
            f"crowd={crowd.name}, crowded_distance={crowded_distance:.2f}m, "
            f"crowd_delay={crowd_delay:.2f}s, final_eta={eta:.2f}s, "
            f"patient_stop=({stop.x:.2f}, {stop.y:.2f})"
        )
        if not self.pending_plans:
            self._finish_planning(serial)

    def _record_plan_failure(
        self, robot_id: str, serial: int, reason: str
    ) -> None:
        if serial != self.request_serial or robot_id not in self.pending_plans:
            return
        self.pending_plans.discard(robot_id)
        self.plan_failures[robot_id] = reason
        self.path_publishers[robot_id].publish(Path())
        self._publish_distance(robot_id, math.nan)
        self._publish_predicted_eta(robot_id, math.nan)
        self._publish_crowd_metrics(robot_id, math.nan, math.nan)
        self.get_logger().warning(f"Candidate {robot_id} excluded: {reason}")
        if not self.pending_plans:
            self._finish_planning(serial)

    def _on_planning_timeout(self, serial: int) -> None:
        if serial != self.request_serial or not self.planning_active:
            return
        for robot_id in tuple(self.pending_plans):
            self.plan_failures[robot_id] = (
                f"planning timeout after {self.planning_timeout:.1f}s"
            )
            self.path_publishers[robot_id].publish(Path())
            self._publish_distance(robot_id, math.nan)
            self._publish_predicted_eta(robot_id, math.nan)
            self._publish_crowd_metrics(robot_id, math.nan, math.nan)
        self.pending_plans.clear()
        self._finish_planning(serial)

    def _finish_planning(self, serial: int) -> None:
        if serial != self.request_serial or not self.planning_active:
            return
        self.planning_active = False
        if self.planning_timer is not None:
            timer = self.planning_timer
            self.planning_timer = None
            timer.cancel()
            self.destroy_timer(timer)

        if not self.plan_results:
            self.selected_robot = ""
            self._publish_selected_robot("")
            detail = "; ".join(
                f"{robot_id}: {reason}"
                for robot_id, reason in self.plan_failures.items()
            )
            self._publish_status("FAILED", detail or "no valid Nav2 path")
            return

        ranked = sorted(
            (
                (robot_id, result[2])
                for robot_id, result in self.plan_results.items()
            ),
            key=lambda item: (item[1], item[0]),
        )
        self.ranked_candidates = [robot_id for robot_id, _ in ranked]
        self.selected_robot = self.ranked_candidates[0]
        self._publish_selected_robot(self.selected_robot)
        dispatch_robot_ids = dispatch_candidates(
            ranked,
            dual_dispatch_enabled=self.dual_dispatch_enabled,
            target_arrival_time=self.target_arrival_time,
            trigger_ratio=self.dual_dispatch_trigger_ratio,
        )
        if len(dispatch_robot_ids) > 1:
            missing_start = [
                robot_id
                for robot_id in dispatch_robot_ids
                if not self.plan_results[robot_id][0].poses
            ]
            if missing_start:
                self.get_logger().warning(
                    "Dual dispatch disabled because start pose is missing: "
                    + ", ".join(missing_start)
                )
                dispatch_robot_ids = dispatch_robot_ids[:1]
        self.dual_dispatch_active = len(dispatch_robot_ids) > 1
        self.dual_dispatch_started_at = (
            time.monotonic() if self.dual_dispatch_active else None
        )
        detail = ", ".join(
            (
                f"{robot_id}={score:.2f}s"
                f"({self.plan_results[robot_id][1]:.2f}m, "
                f"{math.degrees(self.plan_results[robot_id][3]):.1f}deg, "
                f"slow={self.plan_results[robot_id][4]}, "
                f"crowd={self.plan_results[robot_id][8]}, "
                f"delay={self.plan_results[robot_id][7]:.2f}s)"
            )
            for robot_id, score in ranked
        )
        if self.plan_failures:
            detail += "; excluded: " + ", ".join(
                f"{robot_id}({reason})"
                for robot_id, reason in self.plan_failures.items()
            )
        dispatch_detail = "+".join(dispatch_robot_ids)
        if self.dual_dispatch_active:
            trigger_eta = (
                self.target_arrival_time * self.dual_dispatch_trigger_ratio
            )
            detail += (
                "; dual dispatch: fastest ETA "
                f"{ranked[0][1]:.2f}s >= trigger "
                f"{trigger_eta:.2f}s"
            )
        self._publish_status(
            "ASSIGNED",
            f"selected={self.selected_robot}; dispatch={dispatch_detail}; "
            f"{detail}",
        )

        if not self.dispatch_enabled:
            self.get_logger().warning(
                "Dispatch is disabled. Set dispatch_enabled:=true to move "
                "the selected robot."
            )
            return
        for robot_id in dispatch_robot_ids:
            self.dispatch_start_poses[robot_id] = (
                self._capture_dispatch_start_pose(robot_id)
            )
            self._publish_assignment(
                robot_id,
                deepcopy(self.planning_targets[robot_id]),
                role=RobotState.ROLE_AED_DELIVERY,
                mission_suffix="aed",
            )

    def _capture_dispatch_start_pose(self, robot_id: str) -> PoseStamped:
        """Store the planned start position and best known start yaw."""
        start_pose = deepcopy(self.plan_results[robot_id][0].poses[0])
        start_pose.header.frame_id = self.map_frame
        observation = self.observations.get(robot_id)
        if observation is not None:
            start_pose.pose.orientation = deepcopy(
                observation.pose.pose.orientation
            )
        return start_pose

    def _publish_assignment(
        self,
        robot_id: str,
        target: PoseStamped | None,
        *,
        role: int,
        mission_suffix: str,
    ) -> None:
        if target is None:
            self.navigation_active = False
            self._publish_status("FAILED", "assignment target was lost")
            return
        self.assignment_version += 1
        self.assignment_versions[robot_id] = self.assignment_version
        self.dispatched_robots.add(robot_id)
        self.terminal_robots.discard(robot_id)
        self.navigation_active = True
        self.navigation_started_at.pop(robot_id, None)
        if role == RobotState.ROLE_AED_DELIVERY:
            self.navigation_predicted_eta[robot_id] = self.plan_results[
                robot_id
            ][2]
            self.returning_robots.discard(robot_id)
        else:
            self.navigation_predicted_eta.pop(robot_id, None)
            self.returning_robots.add(robot_id)
        self._publish_dispatched_robots()
        assignment = MissionAssignment()
        assignment.mission_id = (
            f"{self.active_request_id}-{mission_suffix}-{robot_id}"
        )
        assignment.event_id = self.active_request_id
        assignment.robot_id = robot_id
        assignment.role = role
        assignment.target = deepcopy(target)
        assignment.assigned_at = self.get_clock().now().to_msg()
        assignment.assignment_version = self.assignment_version
        assignment.cancel_previous = True
        self.assignment_publishers[robot_id].publish(assignment)
        self._start_assignment_ack_timer(
            self.active_request_id,
            robot_id,
            self.assignment_version,
        )
        self._publish_status(
            "DISPATCHING",
            f"assignment v{self.assignment_version} published to {robot_id} "
            f"role={role}",
        )

    def _on_mission_status(self, status: MissionStatus) -> None:
        if status.event_id != self.active_request_id:
            return
        if status.robot_id not in self.dispatched_robots:
            return
        if status.status == MissionStatus.HELPER_ARRIVED:
            self._return_after_helper_handoff(status)
            return
        if status.assignment_version != self.assignment_versions.get(
            status.robot_id
        ):
            return

        self._cancel_assignment_ack_timer(status.robot_id)

        if status.status == MissionStatus.DISPATCHING:
            self._publish_status(
                "DISPATCHING", status.reason or status.robot_id
            )
            return
        if status.status == MissionStatus.EN_ROUTE:
            self.navigation_started_at.setdefault(
                status.robot_id, time.monotonic()
            )
            state = (
                "RETURNING"
                if status.robot_id in self.returning_robots
                else "NAVIGATING"
            )
            self._publish_status(state, f"{status.robot_id} is moving")
            return
        if status.status in (MissionStatus.ARRIVED, MissionStatus.COMPLETED):
            if status.robot_id in self.returning_robots:
                self.returning_robots.discard(status.robot_id)
                self.navigation_started_at.pop(status.robot_id, None)
                self.terminal_robots.add(status.robot_id)
                self._publish_status(
                    "RETURNED",
                    f"{status.robot_id} returned to its dispatch start pose",
                )
                self._finish_if_all_terminal()
                return

            self._record_arrival(status.robot_id)
            self.arrived_robots.add(status.robot_id)
            self.terminal_robots.add(status.robot_id)
            self.selected_robot = status.robot_id
            self._publish_selected_robot(status.robot_id)
            self._publish_status(
                "ARRIVED", f"{status.robot_id} reached the emergency"
            )
            if self.return_after_helper:
                self.awaiting_helper_robots.add(status.robot_id)
                self._publish_status(
                    "HELPER_REQUESTED",
                    f"{status.robot_id} is waiting for helper confirmation",
                )
            if self.dual_dispatch_active:
                self._return_late_robots(status.robot_id)
            self._finish_if_all_terminal()
            return
        if status.status not in {
            MissionStatus.CANCELED,
            MissionStatus.BLOCKED,
            MissionStatus.NETWORK_LOST,
            MissionStatus.NAVIGATION_ERROR,
        }:
            return

        self._handle_navigation_failure(status.robot_id, status.reason)

    def _return_after_helper_handoff(self, status: MissionStatus) -> None:
        """Send the arrived AED robot back after helper handoff completes."""
        robot_id = status.robot_id
        if not self.return_after_helper:
            return
        if robot_id not in self.awaiting_helper_robots:
            self.get_logger().warning(
                f"Ignoring unexpected helper completion from {robot_id}"
            )
            return
        start_pose = self.dispatch_start_poses.get(robot_id)
        if start_pose is None:
            self.awaiting_helper_robots.discard(robot_id)
            self._publish_status(
                "RETURN_FAILED",
                f"{robot_id}: dispatch start pose is unavailable",
            )
            return
        self.awaiting_helper_robots.discard(robot_id)
        # Helper action uses delivery_version + 1. Keep the return assignment
        # strictly newer so its terminal status cannot be confused with helper
        # completion.
        self.assignment_version = max(
            self.assignment_version, int(status.assignment_version)
        )
        self._publish_status(
            "RETURNING",
            f"{robot_id}: helper handoff completed; returning to start",
        )
        self._publish_assignment(
            robot_id,
            deepcopy(start_pose),
            role=RobotState.ROLE_RETURN,
            mission_suffix="helper-return",
        )

    def _start_assignment_ack_timer(
        self, event_id: str, robot_id: str, assignment_version: int
    ) -> None:
        self._cancel_assignment_ack_timer(robot_id)
        self.assignment_ack_timers[robot_id] = self.create_timer(
            self.assignment_ack_timeout,
            lambda: self._on_assignment_ack_timeout(
                event_id, robot_id, assignment_version
            ),
        )

    def _cancel_assignment_ack_timer(self, robot_id: str) -> None:
        timer = self.assignment_ack_timers.pop(robot_id, None)
        if timer is None:
            return
        timer.cancel()
        self.destroy_timer(timer)

    def _on_assignment_ack_timeout(
        self, event_id: str, robot_id: str, assignment_version: int
    ) -> None:
        self._cancel_assignment_ack_timer(robot_id)
        if (
            event_id != self.active_request_id
            or robot_id not in self.dispatched_robots
            or assignment_version != self.assignment_versions.get(robot_id)
            or not self.navigation_active
        ):
            return
        self._handle_navigation_failure(
            robot_id,
            f"mission executor did not acknowledge within "
            f"{self.assignment_ack_timeout:.1f}s",
        )

    def _handle_navigation_failure(
        self, failed_robot: str, reason: str
    ) -> None:
        # DDS or a Nav2 cancel completion can deliver the same terminal state
        # more than once. Never publish two replacement assignments for one
        # failed goal.
        if failed_robot in self.terminal_robots:
            self.get_logger().warning(
                f"Ignoring duplicate terminal status from {failed_robot}: "
                f"{reason}"
            )
            return
        started_at = self.navigation_started_at.pop(failed_robot, None)
        if started_at is not None:
            elapsed = time.monotonic() - started_at
            self.get_logger().warning(
                f"ETA measurement {failed_robot}: aborted after "
                f"{elapsed:.2f}s ({reason or 'unspecified failure'})"
            )
        self.navigation_predicted_eta.pop(failed_robot, None)
        was_returning = failed_robot in self.returning_robots
        self.returning_robots.discard(failed_robot)
        self.failed_robots.add(failed_robot)
        if was_returning:
            self.return_failed_robots.add(failed_robot)
        self.terminal_robots.add(failed_robot)
        self.excluded_robots.add(failed_robot)
        if self.dual_dispatch_active or was_returning:
            self._publish_status(
                "NAVIGATION_ERROR",
                f"{failed_robot}: {reason}; other robot continues",
            )
            self._finish_if_all_terminal()
            return

        next_robot = next(
            (
                robot_id
                for robot_id in self.ranked_candidates
                if robot_id not in self.excluded_robots
            ),
            None,
        )
        if next_robot is None:
            self._finish_if_all_terminal()
            return
        self._publish_status(
            "REASSIGNING",
            f"exclude {failed_robot}; assigning {next_robot}",
        )
        self.dispatch_start_poses[next_robot] = (
            self._capture_dispatch_start_pose(next_robot)
        )
        self._publish_assignment(
            next_robot,
            deepcopy(self.planning_targets[next_robot]),
            role=RobotState.ROLE_AED_DELIVERY,
            mission_suffix="aed",
        )

    def _record_arrival(self, robot_id: str) -> None:
        started_at = self.navigation_started_at.pop(robot_id, None)
        predicted = self.navigation_predicted_eta.pop(robot_id, None)
        if started_at is None or predicted is None:
            return
        actual = time.monotonic() - started_at
        self._publish_actual_eta(robot_id, actual)
        self._publish_eta_result(robot_id, predicted, actual)
        self.get_logger().info(
            f"ETA measurement {robot_id}: predicted={predicted:.2f}s, "
            f"actual={actual:.2f}s, error={actual - predicted:+.2f}s"
        )

    def _monitor_live_replan(self) -> None:
        """Periodically compare fresh remaining ETAs while one robot runs."""
        now = time.monotonic()
        if self.live_replan_active:
            if now - self.live_replan_started_at >= self.live_replan_timeout:
                for robot_id in tuple(self.live_replan_pending):
                    self.live_replan_failures[robot_id] = "live plan timeout"
                self.live_replan_pending.clear()
                self._finish_live_replan(self.live_replan_serial)
            return
        if (
            not self.live_replan_enabled
            or not self.dispatch_enabled
            or not self.navigation_active
            or self.planning_active
            or self.dual_dispatch_active
            or self.live_reassignment_done
            or now < self.live_replan_next_at
        ):
            return

        active = [
            robot_id
            for robot_id in self.dispatched_robots
            if robot_id not in self.terminal_robots
            and robot_id not in self.returning_robots
        ]
        standby = [
            robot_id
            for robot_id in self.robot_ids
            if robot_id not in self.dispatched_robots
            and robot_id not in self.excluded_robots
            and robot_id in self.planning_targets
        ]
        if len(active) != 1 or not standby:
            self.live_replan_next_at = now + self.live_replan_interval
            return

        candidates = [active[0], standby[0]]
        self.live_replan_serial += 1
        serial = self.live_replan_serial
        self.live_replan_active = True
        self.live_replan_started_at = now
        self.live_replan_pending.clear()
        self.live_replan_results.clear()
        self.live_replan_failures.clear()
        self.live_replan_crowd_snapshot = self.crowd_filter.snapshot(now)

        for robot_id in candidates:
            client = self.planner_clients[robot_id]
            if not client.server_is_ready():
                self.live_replan_failures[robot_id] = (
                    f"/{robot_id}/compute_path_to_pose unavailable"
                )
                continue
            goal = ComputePathToPose.Goal()
            goal.goal = deepcopy(self.planning_targets[robot_id])
            goal.goal.header.stamp = self.get_clock().now().to_msg()
            goal.planner_id = self.planner_id
            # Let each planner use its own latest TF pose. This avoids using
            # a stale AMCL sample while the robot is moving.
            goal.use_start = False
            self.live_replan_pending.add(robot_id)
            client.send_goal_async(goal).add_done_callback(
                lambda future, rid=robot_id, run=serial:
                self._on_live_plan_response(rid, run, future)
            )

        if not self.live_replan_pending:
            self._finish_live_replan(serial)

    def _on_live_plan_response(
        self, robot_id: str, serial: int, future
    ) -> None:
        if (
            not self.live_replan_active
            or serial != self.live_replan_serial
            or robot_id not in self.live_replan_pending
        ):
            return
        try:
            handle = future.result()
        except Exception as error:
            self._record_live_plan_failure(
                robot_id, serial, f"request error: {error}"
            )
            return
        if not handle.accepted:
            self._record_live_plan_failure(
                robot_id, serial, "planner rejected live goal"
            )
            return
        handle.get_result_async().add_done_callback(
            lambda result, rid=robot_id, run=serial:
            self._on_live_plan_result(rid, run, result)
        )

    def _on_live_plan_result(self, robot_id: str, serial: int, future) -> None:
        if (
            not self.live_replan_active
            or serial != self.live_replan_serial
            or robot_id not in self.live_replan_pending
        ):
            return
        try:
            wrapped = future.result()
            status = int(wrapped.status)
            path = wrapped.result.path
        except Exception as error:
            self._record_live_plan_failure(
                robot_id, serial, f"result error: {error}"
            )
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._record_live_plan_failure(
                robot_id, serial, f"planner result status={status}"
            )
            return
        if not path.poses:
            self._record_live_plan_failure(
                robot_id, serial, "planner returned empty live path"
            )
            return

        try:
            points = [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in path.poses
            ]
            distance = path_length(points)
            observation = self.observations.get(robot_id)
            initial_yaw = (
                self._quaternion_yaw(observation.pose)
                if observation is not None
                else None
            )
            final_yaw = self._quaternion_yaw(
                self.planning_targets[robot_id]
            )
            base_eta, turn_angle, slowdown_count = path_motion_cost(
                points,
                linear_speed=self.nominal_linear_speed,
                angular_speed=self.nominal_angular_speed,
                slowdown_turn_threshold=self.slowdown_turn_threshold,
                slowdown_penalty=self.slowdown_penalty,
                simplification_tolerance=self.path_simplification_tolerance,
                initial_yaw=initial_yaw,
                final_yaw=final_yaw,
            )
            crowded_distance = path_length_in_polygon(
                points, self.crowd_zone_polygon
            )
            crowd = self.live_replan_crowd_snapshot
            if (
                crowd.fresh
                and crowd.level >= self.crowd_blocking_level
                and crowded_distance > 1e-6
            ):
                raise ValueError(
                    f"crowd stage {crowd.name}: blocked zone intersects "
                    f"{crowded_distance:.2f}m of live path"
                )
            crowd_delay = (
                crowd_delay_seconds(
                    crowded_distance,
                    normal_speed=self.nominal_linear_speed,
                    crowded_speed=self._crowd_speed(crowd),
                )
                if crowd.fresh and crowd.level > 0
                else 0.0
            )
            eta = base_eta + crowd_delay
        except ValueError as error:
            self._record_live_plan_failure(robot_id, serial, str(error))
            return

        result = (
            path,
            distance,
            eta,
            turn_angle,
            slowdown_count,
            base_eta,
            crowded_distance,
            crowd_delay,
            crowd.name,
        )
        self.live_replan_results[robot_id] = result
        self.live_replan_pending.discard(robot_id)
        self.get_logger().info(
            f"Live candidate {robot_id}: remaining={distance:.2f}m, "
            f"eta={eta:.2f}s, crowd={crowd.name}"
        )
        if not self.live_replan_pending:
            self._finish_live_replan(serial)

    def _record_live_plan_failure(
        self, robot_id: str, serial: int, reason: str
    ) -> None:
        if (
            not self.live_replan_active
            or serial != self.live_replan_serial
            or robot_id not in self.live_replan_pending
        ):
            return
        self.live_replan_pending.discard(robot_id)
        self.live_replan_failures[robot_id] = reason
        self.get_logger().warning(
            f"Live candidate {robot_id} unavailable: {reason}"
        )
        if not self.live_replan_pending:
            self._finish_live_replan(serial)

    def _finish_live_replan(self, serial: int) -> None:
        if not self.live_replan_active or serial != self.live_replan_serial:
            return
        self.live_replan_active = False
        now = time.monotonic()
        self.live_replan_next_at = now + self.live_replan_interval
        if not self.navigation_active or self.dual_dispatch_active:
            return
        active = [
            robot_id
            for robot_id in self.dispatched_robots
            if robot_id not in self.terminal_robots
            and robot_id not in self.returning_robots
        ]
        if len(active) != 1:
            return
        current = active[0]
        alternatives = [
            robot_id
            for robot_id in self.robot_ids
            if robot_id != current
            and robot_id not in self.dispatched_robots
            and robot_id not in self.excluded_robots
            and robot_id in self.live_replan_results
        ]
        if not alternatives:
            return
        replacement = min(
            alternatives,
            key=lambda rid: self.live_replan_results[rid][2],
        )
        replacement_eta = self.live_replan_results[replacement][2]
        current_result = self.live_replan_results.get(current)
        current_eta = (
            current_result[2] if current_result is not None else math.inf
        )
        should_switch = should_switch_for_live_eta(
            current_eta,
            replacement_eta,
            minimum_gain=self.live_replan_min_eta_gain,
            switch_ratio=self.live_replan_switch_ratio,
        )
        if not should_switch:
            self.get_logger().info(
                f"Live ETA keeps {current}: current={current_eta:.2f}s, "
                f"standby {replacement}={replacement_eta:.2f}s"
            )
            return

        for robot_id, result in self.live_replan_results.items():
            self.plan_results[robot_id] = result
            self.path_publishers[robot_id].publish(result[0])
            self._publish_distance(robot_id, result[1])
            self._publish_predicted_eta(robot_id, result[2])
            self._publish_crowd_metrics(robot_id, result[6], result[7])
        start_pose = self.dispatch_start_poses.get(current)
        if start_pose is None:
            self.get_logger().error(
                f"Cannot live-reassign {current}: dispatch start pose missing"
            )
            return
        self.dispatch_start_poses[replacement] = (
            self._capture_dispatch_start_pose(replacement)
        )
        self.live_reassignment_done = True
        self.selected_robot = replacement
        self.ranked_candidates = [replacement, current]
        self._publish_selected_robot(replacement)
        reason = self.live_replan_failures.get(current, "")
        self._publish_status(
            "REASSIGNING",
            f"live ETA switch {current}->{replacement}; "
            f"current={current_eta:.2f}s, replacement={replacement_eta:.2f}s"
            + (f"; {reason}" if reason else ""),
        )
        # A return assignment cancels the old delivery goal. The standby then
        # receives the patient target using the newly measured ETA/path.
        self._publish_assignment(
            current,
            deepcopy(start_pose),
            role=RobotState.ROLE_RETURN,
            mission_suffix="live-return",
        )
        self._publish_assignment(
            replacement,
            deepcopy(self.planning_targets[replacement]),
            role=RobotState.ROLE_AED_DELIVERY,
            mission_suffix="live-aed",
        )

    def _monitor_dual_robot_proximity(self) -> None:
        """Return the farther robot when a dual dispatch becomes unsafe."""
        now = time.monotonic()
        if (
            not self.navigation_active
            or not self.dual_dispatch_active
            or self.proximity_return_triggered
            or self.dual_dispatch_started_at is None
            or now - self.dual_dispatch_started_at
            < self.dual_robot_proximity_grace
            or self.planning_target is None
        ):
            self.proximity_close_since = None
            return

        active = [
            robot_id
            for robot_id in self.dispatched_robots
            if robot_id not in self.terminal_robots
            and robot_id not in self.returning_robots
        ]
        if len(active) != 2:
            self.proximity_close_since = None
            return
        observations = {
            robot_id: self.observations.get(robot_id) for robot_id in active
        }
        if any(observation is None for observation in observations.values()):
            self.proximity_close_since = None
            return
        if any(
            now - observation.received_at > self.pose_timeout
            for observation in observations.values()
            if observation is not None
        ):
            self.proximity_close_since = None
            return

        positions = {
            robot_id: (
                observations[robot_id].pose.pose.position.x,
                observations[robot_id].pose.pose.position.y,
            )
            for robot_id in active
        }
        patient = self.planning_target.pose.position
        retreat_robot = proximity_retreat_candidate(
            positions,
            (patient.x, patient.y),
            primary_robot=self.selected_robot,
            threshold=self.dual_robot_proximity_threshold,
        )
        if retreat_robot is None:
            self.proximity_close_since = None
            return
        if self.proximity_close_since is None:
            self.proximity_close_since = now
            return
        if (
            now - self.proximity_close_since
            < self.dual_robot_proximity_confirm
        ):
            return

        keep_robot = next(
            robot_id for robot_id in active if robot_id != retreat_robot
        )
        start_pose = self.dispatch_start_poses.get(retreat_robot)
        if start_pose is None:
            self.get_logger().error(
                f"Cannot proximity-return {retreat_robot}: "
                "dispatch start pose missing"
            )
            self.proximity_return_triggered = True
            return
        separation = math.hypot(
            positions[active[0]][0] - positions[active[1]][0],
            positions[active[0]][1] - positions[active[1]][1],
        )
        self.proximity_return_triggered = True
        self.excluded_robots.add(retreat_robot)
        self.selected_robot = keep_robot
        self._publish_selected_robot(keep_robot)
        self.get_logger().warning(
            f"Robots remained {separation:.2f}m apart for "
            f"{now - self.proximity_close_since:.2f}s; "
            f"returning farther robot {retreat_robot}, "
            f"continuing {keep_robot}"
        )
        self._publish_status(
            "PROXIMITY_AVOIDANCE",
            f"robots={separation:.2f}m; cancel {retreat_robot} and "
            f"return home; {keep_robot} continues",
        )
        self._publish_assignment(
            retreat_robot,
            deepcopy(start_pose),
            role=RobotState.ROLE_RETURN,
            mission_suffix="proximity-return",
        )

    def _return_late_robots(self, winner: str) -> None:
        for robot_id in sorted(self.dispatched_robots):
            if (
                robot_id == winner
                or robot_id in self.terminal_robots
                or robot_id in self.returning_robots
            ):
                continue
            start_pose = self.dispatch_start_poses.get(robot_id)
            if start_pose is None:
                self.get_logger().error(
                    f"Cannot return {robot_id}: dispatch start pose missing"
                )
                self.failed_robots.add(robot_id)
                self.terminal_robots.add(robot_id)
                continue
            started_at = self.navigation_started_at.pop(robot_id, None)
            if started_at is not None:
                elapsed = time.monotonic() - started_at
                self.get_logger().info(
                    f"Canceling late {robot_id} after {elapsed:.2f}s; "
                    f"winner={winner}"
                )
            self._publish_status(
                "RETURNING",
                f"{winner} arrived first; cancel {robot_id} and return home",
            )
            self._publish_assignment(
                robot_id,
                deepcopy(start_pose),
                role=RobotState.ROLE_RETURN,
                mission_suffix="return",
            )

    def _finish_if_all_terminal(self) -> None:
        if self.awaiting_helper_robots:
            return
        if not self.dispatched_robots or not self.dispatched_robots.issubset(
            self.terminal_robots
        ):
            return
        self.navigation_active = False
        self._publish_dispatched_robots()
        arrived = ",".join(sorted(self.arrived_robots)) or "none"
        failed = ",".join(sorted(self.failed_robots)) or "none"
        if self.return_failed_robots:
            return_failed = ",".join(sorted(self.return_failed_robots))
            self._publish_status(
                "RETURN_FAILED",
                f"emergency reached by {arrived}; return failed="
                f"{return_failed}",
            )
        elif self.arrived_robots:
            self._publish_status(
                "COMPLETED",
                f"{self.active_request_id}; arrived={arrived}; "
                f"failed={failed}",
            )
        else:
            self._publish_status(
                "FAILED",
                f"all candidates failed; failed={failed}",
            )

    def _publish_automatic_request(self) -> None:
        if self._automatic_request_timer is None:
            return
        self._automatic_request_timer.cancel()
        self._automatic_request_timer = None
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = float(
            self.get_parameter("initial_target_x").value
        )
        pose.pose.position.y = float(
            self.get_parameter("initial_target_y").value
        )
        yaw = float(self.get_parameter("initial_target_yaw").value)
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        self._on_request(pose)

    def _publish_target_marker(self, target: PoseStamped) -> None:
        marker = Marker()
        marker.header = deepcopy(target.header)
        marker.ns = "emergency_target"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = deepcopy(target.pose)
        marker.pose.position.z = 0.15
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.30
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.05
        marker.color.a = 0.95
        self.target_marker_publisher.publish(marker)

    def _publish_robot_markers(self) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        colors = ((0.1, 0.45, 1.0), (0.1, 0.9, 0.25))
        for index, robot_id in enumerate(self.robot_ids):
            observation = self.observations.get(robot_id)
            if observation is None:
                continue
            position = observation.pose.pose.position
            selected = robot_id == self.selected_robot
            dispatched = (
                robot_id in self.dispatched_robots
                and robot_id not in self.terminal_robots
            )
            returning = robot_id in self.returning_robots
            body = Marker()
            body.header.frame_id = self.map_frame
            body.header.stamp = stamp
            body.ns = "robot_positions"
            body.id = index
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose = deepcopy(observation.pose.pose)
            body.pose.position.z = 0.12
            body.scale.x = 0.42 if selected or dispatched else 0.32
            body.scale.y = 0.42 if selected or dispatched else 0.32
            body.scale.z = 0.24
            if returning:
                body.color.r, body.color.g, body.color.b = (0.7, 0.2, 1.0)
            elif selected:
                body.color.r, body.color.g, body.color.b = (1.0, 0.8, 0.0)
            elif dispatched:
                body.color.r, body.color.g, body.color.b = (1.0, 0.35, 0.0)
            else:
                body.color.r, body.color.g, body.color.b = colors[index]
            body.color.a = 0.9
            markers.markers.append(body)

            label = Marker()
            label.header = deepcopy(body.header)
            label.ns = "robot_labels"
            label.id = 100 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = position.x
            label.pose.position.y = position.y
            label.pose.position.z = 0.55
            label.pose.orientation.w = 1.0
            label.scale.z = 0.22
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            if returning:
                suffix = " [RETURNING]"
            elif selected:
                suffix = " [PRIMARY]"
            elif dispatched:
                suffix = " [DISPATCHED]"
            else:
                suffix = ""
            label.text = f"{robot_id}{suffix}"
            markers.markers.append(label)
        self.robot_marker_publisher.publish(markers)
        self._publish_crowd_state(
            self.crowd_filter.snapshot(time.monotonic())
        )

    def _publish_selected_robot(self, robot_id: str) -> None:
        message = String()
        message.data = robot_id
        self.selected_publisher.publish(message)

    def _publish_dispatched_robots(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "dual_dispatch": self.dual_dispatch_active,
                "primary_robot": self.selected_robot,
                "dispatched_robots": sorted(self.dispatched_robots),
                "active_robots": sorted(
                    self.dispatched_robots - self.terminal_robots
                ),
                "returning_robots": sorted(self.returning_robots),
                "return_failed_robots": sorted(
                    self.return_failed_robots
                ),
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self.dispatched_publisher.publish(message)

    def _publish_distance(self, robot_id: str, distance: float) -> None:
        message = Float32()
        message.data = float(distance)
        self.distance_publishers[robot_id].publish(message)

    def _publish_predicted_eta(self, robot_id: str, eta: float) -> None:
        message = Float32()
        message.data = float(eta)
        self.predicted_eta_publishers[robot_id].publish(message)

    def _publish_actual_eta(self, robot_id: str, elapsed: float) -> None:
        message = Float32()
        message.data = float(elapsed)
        self.actual_eta_publishers[robot_id].publish(message)

    def _publish_crowd_metrics(
        self, robot_id: str, crowded_distance: float, delay: float
    ) -> None:
        distance_message = Float32()
        distance_message.data = float(crowded_distance)
        self.crowded_distance_publishers[robot_id].publish(distance_message)
        delay_message = Float32()
        delay_message.data = float(delay)
        self.crowd_delay_publishers[robot_id].publish(delay_message)

    def _publish_crowd_state(self, snapshot: CrowdSnapshot) -> None:
        message = String()
        message.data = json.dumps(
            {
                "age_sec": (
                    round(snapshot.age_sec, 3)
                    if math.isfinite(snapshot.age_sec)
                    else None
                ),
                "fresh": snapshot.fresh,
                "level": snapshot.level,
                "level_name": snapshot.name,
                "person_count": snapshot.person_count,
                "raw_level": self.raw_crowd_level,
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self.crowd_state_publisher.publish(message)
        self._publish_crowd_markers(snapshot)
        self._publish_keepout_mask(snapshot)

    def _crowd_speed(self, snapshot: CrowdSnapshot) -> float:
        if snapshot.fresh and snapshot.level > 0:
            return self.crowd_level_speeds[snapshot.level]
        return self.nominal_linear_speed

    def _publish_keepout_mask(
        self, snapshot: CrowdSnapshot, *, force: bool = False
    ) -> None:
        """Publish a Nav2 keepout mask while the camera stage is BLOCKED."""
        active = (
            snapshot.fresh
            and snapshot.level >= self.crowd_blocking_level
        )
        if self.latest_map is None:
            return
        if not force and active == self.keepout_active:
            return

        source = self.latest_map
        mask = OccupancyGrid()
        mask.header = deepcopy(source.header)
        mask.header.frame_id = self.map_frame
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.info = deepcopy(source.info)
        width = int(mask.info.width)
        height = int(mask.info.height)
        resolution = float(mask.info.resolution)
        origin_x = float(mask.info.origin.position.x)
        origin_y = float(mask.info.origin.position.y)
        data = [0] * (width * height)
        if active:
            for row in range(height):
                y = origin_y + (row + 0.5) * resolution
                for column in range(width):
                    x = origin_x + (column + 0.5) * resolution
                    if point_to_polygon_distance(
                        (x, y), self.crowd_zone_polygon
                    ) <= self.crowd_keepout_margin:
                        data[row * width + column] = 100
        mask.data = data

        info = CostmapFilterInfo()
        info.header.frame_id = self.map_frame
        info.header.stamp = mask.header.stamp
        info.type = 0
        info.filter_mask_topic = self.crowd_keepout_mask_topic
        info.base = 0.0
        info.multiplier = 1.0
        self.filter_info_publisher.publish(info)
        self.keepout_mask_publisher.publish(mask)
        self.keepout_active = active
        self.get_logger().info(
            "Crowd keepout mask "
            + ("enabled" if active else "cleared")
        )

    def _publish_crowd_markers(self, snapshot: CrowdSnapshot) -> None:
        """Show the monitored map polygon and current stage in RViz."""
        stamp = self.get_clock().now().to_msg()
        vertices = [
            Point(x=x, y=y, z=0.03)
            for x, y in self.crowd_zone_polygon
        ]
        center_x = sum(point.x for point in vertices) / len(vertices)
        center_y = sum(point.y for point in vertices) / len(vertices)
        colors = {
            -1: (0.55, 0.55, 0.55),
            0: (0.10, 0.85, 0.20),
            1: (1.00, 0.85, 0.05),
            2: (1.00, 0.40, 0.05),
            3: (0.95, 0.05, 0.05),
        }
        red, green, blue = colors.get(snapshot.level, colors[3])

        area = Marker()
        area.header.frame_id = self.map_frame
        area.header.stamp = stamp
        area.ns = "crowd_zone"
        area.id = 0
        area.type = Marker.TRIANGLE_LIST
        area.action = Marker.ADD
        area.pose.orientation.w = 1.0
        center = Point(x=center_x, y=center_y, z=0.03)
        for index, vertex in enumerate(vertices):
            area.points.extend(
                [center, vertex, vertices[(index + 1) % len(vertices)]]
            )
        area.color.r = red
        area.color.g = green
        area.color.b = blue
        area.color.a = 0.28

        outline = Marker()
        outline.header = deepcopy(area.header)
        outline.ns = "crowd_zone"
        outline.id = 1
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.points = vertices + [vertices[0]]
        outline.scale.x = 0.06
        outline.color.r = red
        outline.color.g = green
        outline.color.b = blue
        outline.color.a = 0.95

        label = Marker()
        label.header = deepcopy(area.header)
        label.ns = "crowd_zone"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(x=center_x, y=center_y, z=0.35)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.22
        label.color.r = red
        label.color.g = green
        label.color.b = blue
        label.color.a = 1.0
        label.text = (
            f"ALLEY: {snapshot.name}\npeople={snapshot.person_count}"
        )
        markers = MarkerArray()
        markers.markers = [area, outline, label]
        self.crowd_marker_publisher.publish(markers)

    def _publish_eta_result(
        self, robot_id: str, predicted: float, actual: float
    ) -> None:
        stamp = self.get_clock().now()
        candidate = self.plan_results.get(robot_id)
        message = String()
        message.data = json.dumps(
            {
                "request_id": self.active_request_id,
                "robot_id": robot_id,
                "predicted_eta_sec": round(predicted, 3),
                "actual_arrival_sec": round(actual, 3),
                "error_sec": round(actual - predicted, 3),
                "status": "ARRIVED",
                "stamp_sec": round(stamp.nanoseconds / 1e9, 9),
                "base_eta_sec": (
                    round(candidate[5], 3) if candidate is not None else None
                ),
                "crowd_level": (
                    candidate[8] if candidate is not None else "UNKNOWN"
                ),
                "crowded_distance_m": (
                    round(candidate[6], 3) if candidate is not None else None
                ),
                "crowd_delay_sec": (
                    round(candidate[7], 3) if candidate is not None else None
                ),
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self.eta_result_publisher.publish(message)

    def _publish_status(self, state: str, detail: str) -> None:
        self.state = state
        message = String()
        message.data = (
            f"{state}|request={self.active_request_id}|"
            f"selected={self.selected_robot}|{detail}"
        )
        self.status_publisher.publish(message)
        self.get_logger().info(message.data)

    @staticmethod
    def _finite_pose(pose: PoseStamped) -> bool:
        values = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        return all(math.isfinite(value) for value in values)

    @staticmethod
    def _quaternion_yaw(pose: PoseStamped) -> float:
        orientation = pose.pose.orientation
        return math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

    def _project_undocked_start(self, pose: PoseStamped) -> PoseStamped:
        """Move a docked pose backward to the expected post-undock pose."""
        result = deepcopy(pose)
        yaw = self._quaternion_yaw(result)
        result.pose.position.x -= self.docked_start_offset * math.cos(yaw)
        result.pose.position.y -= self.docked_start_offset * math.sin(yaw)
        return result


def main(args=None) -> None:
    """Run the emergency mission manager until ROS shuts down."""
    rclpy.init(args=args)
    node = EmergencyMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
