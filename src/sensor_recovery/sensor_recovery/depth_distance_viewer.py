"""Display compressedDepth distance and the follower's obstacle ROI."""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from sensor_recovery.depth_decode import decode_compressed_depth
from sensor_recovery.depth_metrics import (
    compute_depth_region_metrics,
    format_distance,
)
from sensor_recovery.path_follow_control import (
    DepthSafetyResult,
    evaluate_depth_safety,
)


class DepthDistanceViewer(Node):
    """Show a colour depth view without publishing any motion command."""

    def __init__(self) -> None:
        super().__init__("depth_distance_viewer")
        self.declare_parameter(
            "depth_topic", "oakd/stereo/image_raw/compressedDepth"
        )
        self.declare_parameter("obstacle_distance_m", 0.65)
        self.declare_parameter("obstacle_pixel_ratio", 0.03)
        self.declare_parameter("min_valid_pixel_ratio", 0.20)
        self.declare_parameter("noise_valid_pixel_ratio", 0.60)
        self.declare_parameter("center_roi_size_px", 80)
        self.declare_parameter("display_max_distance_m", 3.0)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.obstacle_distance_m = float(
            self.get_parameter("obstacle_distance_m").value
        )
        self.obstacle_pixel_ratio = float(
            self.get_parameter("obstacle_pixel_ratio").value
        )
        self.min_valid_pixel_ratio = float(
            self.get_parameter("min_valid_pixel_ratio").value
        )
        self.noise_valid_pixel_ratio = float(
            self.get_parameter("noise_valid_pixel_ratio").value
        )
        self.center_roi_size_px = int(
            self.get_parameter("center_roi_size_px").value
        )
        self.display_max_distance_m = float(
            self.get_parameter("display_max_distance_m").value
        )
        self.exit_requested = False
        self.frame_count = 0
        self.stats_started_at = time.monotonic()
        self.last_console_at = 0.0
        self.window_name = "Robot depth distance (q/Esc: quit)"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.subscription = self.create_subscription(
            CompressedImage,
            self.depth_topic,
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.get_logger().warning(
            "Viewer only: no cmd_vel publisher is created; the robot will not move"
        )

    def _on_depth(self, message: CompressedImage) -> None:
        try:
            depth = decode_compressed_depth(message.data)
        except Exception as error:
            self.get_logger().error(f"compressed depth conversion failed: {error}")
            return

        received_at = self.get_clock().now().nanoseconds * 1e-9
        header_at = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        header_age = received_at - header_at
        height, width = depth.shape
        half = max(1, self.center_roi_size_px // 2)
        roi = (
            max(0, width // 2 - half),
            max(0, height // 2 - half),
            min(width, width // 2 + half),
            min(height, height // 2 + half),
        )
        metrics = compute_depth_region_metrics(
            depth, roi, self.obstacle_distance_m
        )
        result = evaluate_depth_safety(
            depth,
            self.obstacle_distance_m,
            self.obstacle_pixel_ratio,
            self.min_valid_pixel_ratio,
            roi,
            self.noise_valid_pixel_ratio,
        )

        patch_half = 10
        center_roi = (
            max(0, width // 2 - patch_half),
            max(0, height // 2 - patch_half),
            min(width, width // 2 + patch_half),
            min(height, height // 2 + patch_half),
        )
        center_metrics = compute_depth_region_metrics(
            depth, center_roi, self.obstacle_distance_m
        )
        self.frame_count += 1
        elapsed = max(time.monotonic() - self.stats_started_at, 1e-6)
        rate_hz = self.frame_count / elapsed
        self._show(
            depth,
            roi,
            center_roi,
            result,
            metrics,
            center_metrics,
            rate_hz,
            header_age,
        )
        now = time.monotonic()
        if now - self.last_console_at >= 0.5:
            self.last_console_at = now
            self.get_logger().info(
                f"DEPTH_DISTANCE result={result.value} "
                f"center={format_distance(center_metrics.median_m)} "
                f"roi_p05={format_distance(metrics.p05_m)} "
                f"valid={metrics.valid_ratio * 100:.1f}% "
                f"under_{self.obstacle_distance_m:.2f}m="
                f"{metrics.close_ratio * 100:.1f}% "
                f"rate={rate_hz:.1f}Hz header_age={header_age:.2f}s"
            )

    def _show(
        self,
        depth,
        roi,
        center_roi,
        result,
        metrics,
        center_metrics,
        rate_hz,
        header_age,
    ) -> None:
        maximum_mm = max(self.display_max_distance_m * 1000.0, 1.0)
        clipped = np.clip(depth.astype(np.float32), 0.0, maximum_mm)
        gray = np.uint8(255.0 - clipped * 255.0 / maximum_mm)
        view = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
        view[depth == 0] = (0, 0, 0)
        colour = (
            (0, 255, 0)
            if result == DepthSafetyResult.CLEAR
            else (0, 0, 255)
        )
        cv2.rectangle(view, roi[:2], roi[2:], colour, 3)
        cv2.rectangle(view, center_roi[:2], center_roi[2:], (255, 255, 255), 2)
        lines = [
            f"VERDICT: {result.value}",
            f"Center 20x20 median: {format_distance(center_metrics.median_m)}",
            "ROI p05 / median: "
            f"{format_distance(metrics.p05_m)} / "
            f"{format_distance(metrics.median_m)}",
            f"Detected close median: {format_distance(metrics.close_median_m)}",
            f"ROI valid: {metrics.valid_ratio * 100:.1f}% "
            f"(noise < {self.noise_valid_pixel_ratio * 100:.1f}%, "
            f"invalid < {self.min_valid_pixel_ratio * 100:.1f}%)",
            f"ROI < {self.obstacle_distance_m:.2f}m: "
            f"{metrics.close_ratio * 100:.1f}% "
            f"(stop >= {self.obstacle_pixel_ratio * 100:.1f}%)",
            f"Rate: {rate_hz:.1f}Hz   header age: {header_age:.2f}s",
            "Green=clear, Red=stop, White=center measurement",
            "Viewer only: NO cmd_vel output",
        ]
        for index, line in enumerate(lines):
            y = 28 + index * 27
            cv2.putText(
                view,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 0, 0),
                4,
                cv2.LINE_AA,
            )
            cv2.putText(
                view,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        cv2.imshow(self.window_name, view)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            self.exit_requested = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthDistanceViewer()
    try:
        while rclpy.ok() and not node.exit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
