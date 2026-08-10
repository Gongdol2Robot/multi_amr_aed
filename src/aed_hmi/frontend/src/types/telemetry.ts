/**
 * 백엔드 backend/domain/models.py 와 1:1 이다.
 * 한쪽을 고치면 다른 쪽도 고쳐야 한다. 필드 이름을 임의로 바꾸지 않는다.
 *
 * 문자열 유니온으로 둔 값들은 aed_interfaces 의 .msg 상수에서 온다.
 * 숫자가 아니라 이름으로 오므로 화면 코드가 읽힌다.
 */

export type MissionState =
  | 'assigned'
  | 'dispatching'
  | 'en_route'
  | 'arrived'
  | 'completed'
  | 'canceled'
  | 'blocked'
  | 'network_lost'
  | 'navigation_error'
  | 'recovery_wait'
  | 'recovery_resumed'
  | 'helper_requested'
  | 'helper_en_route'
  | 'helper_arrived';

export type RobotAvailability =
  | 'available'
  | 'busy'
  | 'blocked'
  | 'network_lost'
  | 'navigation_error'
  | 'localization_error'
  | 'low_battery'
  | 'emergency_stop'
  | 'unavailable';

export type RobotRole =
  | 'none'
  | 'aed_delivery'
  | 'helper_request'
  | 'guide'
  | 'return';

export type EventStatus =
  | 'detected'
  | 'confirmed'
  | 'dispatched'
  | 'resolved'
  | 'canceled';

export type LidarState =
  | 'UNKNOWN'
  | 'STARTING'
  | 'ALIVE'
  | 'FAULT'
  | 'RECOVERING';

export type FallbackState =
  | 'UNKNOWN'
  | 'IDLE'
  | 'STARTING'
  | 'ACTIVE'
  | 'BLOCKED'
  | 'SUCCEEDED'
  | 'FAILED';

/** 지도 좌표를 그림 좌표로 바꾸는 계수. /api/map/meta 가 준다. */
export interface MapMeta {
  width: number;
  height: number;
  resolution: number;
  origin_x: number;
  origin_y: number;
}

export interface Point2D {
  x: number;
  y: number;
}

export interface CrowdZoneSnapshot {
  zone_id: string;
  polygon: Point2D[];
  level: number;
  level_name: string;
  person_count: number;
  fresh: boolean;
  age_sec: number | null;
}

export interface RobotSnapshot {
  robot_id: string;
  stamp: number;
  position: Point2D;
  yaw_deg: number;
  battery_percentage: number;
  availability: RobotAvailability;
  role: RobotRole;
  mission_id: string;
  is_docked: boolean;
  network_ok: boolean;
  localization_ok: boolean;
  nav2_ok: boolean;
  emergency_stop: boolean;
  path_valid: boolean;
  estimated_path_cost: number;
  last_heartbeat: number;
  detail: string;
  speed_mps: number;
  heartbeat_age_s: number;
  lidar_state: LidarState;
  lidar_ok: boolean | null;
  fallback_state: FallbackState;
  /** 백엔드가 계산한다. 화면에서 다시 판단하지 않는다. */
  healthy: boolean;
}

export interface EmergencyEventSnapshot {
  event_id: string;
  detected_at: number;
  location: Point2D;
  frame_id: string;
  confidence: number;
  consecutive_detections: number;
  status: EventStatus;
  source_id: string;
  camera_id: string;
  zone_id: string;
  crowd_level: number;
  location_source: string;
  location_valid: boolean;
}

export interface MissionSummary {
  mission_id: string;
  event_id: string;
  robot_id: string;
  target: Point2D;
  called_at: number;
  dispatched_at: number | null;
  arrived_at: number | null;
  final_state: MissionState;
  assignment_version: number;
  reassignment_count: number;
  failure_reasons: string[];
  /** 신고에서 AED 도착까지. 도착 못 했으면 null. */
  response_seconds: number | null;

  /** 도착까지 남은 예상 시간(초). 이동 중인 임무에만 채워진다. */
  eta_seconds: number | null;
  /** 도착 예상 시각(epoch 초). 서버가 확정해 준 값이라 화면은 그대로 쓴다. */
  eta_at: number | null;
  /** 배정 순간 중앙제어가 계산한 고정 ETA(초). */
  initial_eta_seconds: number | null;
  /** 배정 순간 예상했던 고정 도착 시각(epoch 초). */
  initial_eta_at: number | null;
  /** 완료 후 중앙제어가 /emergency/eta/result에 남긴 최초 예상 주행시간. */
  predicted_eta_seconds: number | null;
  /** 완료 후 측정된 실제 Nav2 주행시간. */
  actual_travel_seconds: number | null;
  /** |실제-예상| / 실제 × 100. */
  eta_error_rate_percent: number | null;
}

export interface MissionEvent {
  mission_id: string;
  event_id: string;
  robot_id: string;
  assignment_version: number;
  state: MissionState;
  stamp: number;
  reason: string;
}

export interface StreamHealth {
  stream_id: string;
  label: string;
  /** 'robot' 또는 'webcam' */
  kind: string;
  online: boolean;
  fps: number;
  last_frame_at: number | null;
  detections: number;
}

export interface SystemSnapshot {
  stamp: number;
  robots: RobotSnapshot[];
  active_event: EmergencyEventSnapshot | null;
  active_missions: MissionSummary[];
  streams: StreamHealth[];
  crowd_zone: CrowdZoneSnapshot | null;
  ros_connected: boolean;
}

export interface ResponseTimeStats {
  total: number;
  arrived: number;
  avg_seconds: number | null;
  min_seconds: number | null;
  max_seconds: number | null;
}

/** 출동 한 건의 예상 대 실제. 백엔드 domain/models.py 의 EtaRecord 와 1:1. */
export interface EtaRecord {
  request_id: string;
  robot_id: string;
  predicted_sec: number;
  actual_sec: number;
  /** 양수면 예상보다 늦게 도착했다는 뜻. */
  error_sec: number;
  status: string;
  stamp: number;
}

export interface EtaAccuracyStats {
  total: number;
  /** 평균 오차. 부호가 상쇄되므로 이것만 보면 안 된다. */
  avg_error_sec: number | null;
  /** 절대 오차 평균. 실제 정확도는 이쪽이다. */
  avg_abs_error_sec: number | null;
  max_abs_error_sec: number | null;
  avg_predicted_sec: number | null;
  avg_actual_sec: number | null;
  /** 예상보다 늦은 건수. 관제에서 문제가 되는 것은 이쪽뿐이다. */
  late_count: number;
  recent: EtaRecord[];
}
