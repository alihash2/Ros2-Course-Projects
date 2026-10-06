#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from ms05_qr_vision.msg import QrDetection

_PAYLOAD_PREFIX = 'ms05'


class QrDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')

        self.declare_parameter('frame_id', '')
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('info_topic', 'camera/camera_info')
        self.declare_parameter('range_min', 0.4)
        self.declare_parameter('range_max', 3.0)
        self.declare_parameter('yaw_tolerance', 0.25)
        self.declare_parameter('center_tolerance', 0.15)
        self.declare_parameter('centered_frames', 5)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('tag_size', 0.34)

        self._frame_id = self.get_parameter('frame_id').value
        self._range_min = float(self.get_parameter('range_min').value)
        self._range_max = float(self.get_parameter('range_max').value)
        self._yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self._center_tolerance = float(self.get_parameter('center_tolerance').value)
        self._centered_frames = int(self.get_parameter('centered_frames').value)
        self._publish_rate = max(0.1, float(self.get_parameter('publish_rate').value))
        self._tag_size = float(self.get_parameter('tag_size').value)

        half = self._tag_size / 2.0
        # object points: TL, TR, BR, BL with x right, y up, z out of the tag face
        self._objp = np.array(
            [(-half, half, 0.0), (half, half, 0.0), (half, -half, 0.0), (-half, -half, 0.0)],
            dtype=np.float64,
        )

        self._bridge = CvBridge()
        self._detector = cv2.QRCodeDetector()
        self._last_image = None
        self._frame_seq = 0
        self._processed_seq = -1
        self._camera_info = None
        self._centered_streak = 0
        self._last_log_ns = 0
        self._last_warn_ns = 0

        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image, 10
        )
        self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value, self._on_camera_info, 10
        )
        self._publisher = self.create_publisher(QrDetection, 'qr/detections', 10)
        self.create_timer(1.0 / self._publish_rate, self._process)

    def _on_image(self, msg: Image):
        self._last_image = msg
        self._frame_seq += 1

    def _on_camera_info(self, msg: CameraInfo):
        self._camera_info = msg

    def _throttled(self, msg: str, period_ns: int, warn: bool = False):
        now = self.get_clock().now().nanoseconds
        last = self._last_warn_ns if warn else self._last_log_ns
        if now - last >= period_ns:
            if warn:
                self._last_warn_ns = now
                self.get_logger().warning(msg)
            else:
                self._last_log_ns = now
                self.get_logger().info(msg)

    @staticmethod
    def _order_corners(pts: np.ndarray):
        u = pts[:, 0]
        v = pts[:, 1]
        idx = [
            int(np.argmin(u + v)),
            int(np.argmin(v - u)),
            int(np.argmax(u + v)),
            int(np.argmax(v - u)),
        ]
        if len(set(idx)) != 4:
            return None
        return pts[idx]

    @staticmethod
    def _classify(result):
        decoded = None
        points = None
        for el in result:
            if isinstance(el, np.ndarray):
                if el.ndim == 3 and el.shape[-1] == 2:
                    points = el
                elif not (el.ndim == 2 and el.dtype == np.uint8):
                    decoded = el
            elif isinstance(el, (str, list, tuple)):
                decoded = el
        return decoded, points

    @staticmethod
    def _decoded_str(decoded):
        if decoded is None:
            return None
        if isinstance(decoded, str):
            return decoded or None
        if isinstance(decoded, np.ndarray):
            if decoded.size == 0:
                return None
            return str(decoded.reshape(-1)[0]) or None
        if isinstance(decoded, (list, tuple)):
            if len(decoded) == 0:
                return None
            return str(decoded[0]) or None
        return str(decoded) or None

    @staticmethod
    def _gray_from_msg(msg: Image, bridge: CvBridge):
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if img.ndim == 2:
            return img
        codes = {
            'bgr8': cv2.COLOR_BGR2GRAY,
            'rgb8': cv2.COLOR_RGB2GRAY,
            'bgra8': cv2.COLOR_BGRA2GRAY,
            'rgba8': cv2.COLOR_RGBA2GRAY,
        }
        code = codes.get(msg.encoding, cv2.COLOR_BGR2GRAY)
        return cv2.cvtColor(img, code)

    def _process(self):
        if self._last_image is None or self._camera_info is None:
            return
        if self._frame_seq == self._processed_seq:
            return
        self._processed_seq = self._frame_seq
        try:
            self._process_frame(self._last_image, self._camera_info)
        except Exception as exc:  # noqa: BLE001 - degrade gracefully, never crash
            self._centered_streak = 0
            self._throttled(
                f'detection failed on frame: {exc}', 5_000_000_000, warn=True
            )

    def _process_frame(self, image: Image, info: CameraInfo):
        gray = self._gray_from_msg(image, self._bridge)
        decoded, points = self._classify(self._detector.detectAndDecode(gray))
        decoded = self._decoded_str(decoded)
        if decoded is None or points is None or np.asarray(points).size < 8:
            self._centered_streak = 0
            return

        parts = decoded.split(':')
        if len(parts) != 3 or parts[0] != _PAYLOAD_PREFIX:
            self._centered_streak = 0
            self._throttled(
                f'unexpected payload {decoded!r}, ignoring', 10_000_000_000, warn=True
            )
            return
        try:
            marker_id = int(parts[2])
        except ValueError:
            self._centered_streak = 0
            self._throttled(
                f'unparsable marker index in payload {decoded!r}', 10_000_000_000, warn=True
            )
            return

        corners = self._order_corners(np.asarray(points, dtype=np.float64).reshape(-1, 2))
        if corners is None:
            self._centered_streak = 0
            return

        k = np.array(info.k, dtype=np.float64).reshape(3, 3)
        dist = np.array(info.d, dtype=np.float64) if info.d else np.zeros(5)
        try:
            ok, rvec, tvec = cv2.solvePnP(
                self._objp, corners, k, dist, flags=cv2.SOLVEPNP_ITERATIVE
            )
        except cv2.error as exc:
            self._centered_streak = 0
            self._throttled(f'solvePnP failed: {exc}', 5_000_000_000, warn=True)
            return
        if not ok:
            self._centered_streak = 0
            return

        position = tvec.flatten()
        x, y, z = (float(position[0]), float(position[1]), float(position[2]))
        if z <= 1e-6:
            self._centered_streak = 0
            return

        rng = float(np.linalg.norm(position))
        bearing = math.atan2(x, z)
        rot, _ = cv2.Rodrigues(rvec)
        nx, nz = float(rot[0, 2]), float(rot[2, 2])
        yaw_error = math.atan2(nx, -nz)
        confidence = self._sharpness(gray, corners)

        in_range = self._range_min <= rng <= self._range_max
        in_yaw = abs(yaw_error) <= self._yaw_tolerance
        in_bearing = abs(bearing) <= self._center_tolerance
        if in_range and in_yaw and in_bearing:
            self._centered_streak += 1
        else:
            self._centered_streak = 0
        centered = self._centered_streak >= self._centered_frames

        msg = QrDetection()
        now = self.get_clock().now().to_msg()
        frame_id = self._frame_id or info.header.frame_id
        msg.header.stamp = now
        msg.header.frame_id = frame_id
        msg.marker_id = marker_id
        msg.payload = decoded
        msg.room = parts[1]
        msg.camera_point = Point(x=x, y=y, z=z)
        msg.range = rng
        msg.bearing = bearing
        msg.yaw_error = yaw_error
        msg.confidence = confidence
        msg.centered = centered
        self._publisher.publish(msg)

        self._throttled(
            f'marker {marker_id} room {parts[1]} range {rng:.2f}m '
            f'bearing {bearing:+.2f}rad yaw {yaw_error:+.2f}rad '
            f'conf {confidence:.2f} centered {centered}',
            2_000_000_000,
        )

    def _sharpness(self, gray: np.ndarray, corners: np.ndarray) -> float:
        x0 = max(0, int(np.floor(corners[:, 0].min())))
        x1 = min(gray.shape[1], int(np.ceil(corners[:, 0].max())) + 1)
        y0 = max(0, int(np.floor(corners[:, 1].min())))
        y1 = min(gray.shape[0], int(np.ceil(corners[:, 1].max())) + 1)
        if x1 - x0 < 5 or y1 - y0 < 5:
            return 1.0
        patch = gray[y0:y1, x0:x1]
        var = float(cv2.Laplacian(patch, cv2.CV_64F).var())
        return float(np.clip(var / 400.0, 0.0, 1.0))


def main(args=None):
    rclpy.init(args=args)
    node = QrDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
