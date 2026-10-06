#!/usr/bin/env python3
import os
import tempfile

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from ms05_auto_mapping.msg import QrTagRecord
from ms05_auto_mapping.srv import QueryQrTag, RecordQrTag


def _time_to_dict(t: Time):
    return {'sec': t.sec, 'nanosec': t.nanosec}


def _time_from_dict(d):
    return Time(sec=int(d.get('sec', 0)), nanosec=int(d.get('nanosec', 0)))


def _pose_to_dict(p: PoseStamped):
    return {
        'header': {'stamp': _time_to_dict(p.header.stamp), 'frame_id': p.header.frame_id},
        'pose': {
            'position': {'x': p.pose.position.x, 'y': p.pose.position.y, 'z': p.pose.position.z},
            'orientation': {
                'x': p.pose.orientation.x,
                'y': p.pose.orientation.y,
                'z': p.pose.orientation.z,
                'w': p.pose.orientation.w,
            },
        },
    }


def _pose_from_dict(d):
    p = PoseStamped()
    d = d or {}
    header = d.get('header', {})
    p.header.stamp = _time_from_dict(header.get('stamp', {}))
    p.header.frame_id = header.get('frame_id', '')
    pose = d.get('pose', {})
    pos = pose.get('position', {})
    ori = pose.get('orientation', {})
    p.pose.position.x = float(pos.get('x', 0.0))
    p.pose.position.y = float(pos.get('y', 0.0))
    p.pose.position.z = float(pos.get('z', 0.0))
    p.pose.orientation.x = float(ori.get('x', 0.0))
    p.pose.orientation.y = float(ori.get('y', 0.0))
    p.pose.orientation.z = float(ori.get('z', 0.0))
    p.pose.orientation.w = float(ori.get('w', 0.0))
    return p


def _record_to_dict(r: QrTagRecord):
    return {
        'marker_id': r.marker_id,
        'header': {'stamp': _time_to_dict(r.header.stamp), 'frame_id': r.header.frame_id},
        'tag_map_pose': _pose_to_dict(r.tag_map_pose),
        'robot_map_pose': _pose_to_dict(r.robot_map_pose),
        'recorded_at': _time_to_dict(r.recorded_at),
        'source_robot': r.source_robot,
        'range_at_capture': r.range_at_capture,
        'room': r.room,
    }


def _record_from_dict(d):
    r = QrTagRecord()
    r.marker_id = int(d.get('marker_id', -1))
    header = d.get('header', {})
    r.header.stamp = _time_from_dict(header.get('stamp', {}))
    r.header.frame_id = header.get('frame_id', '')
    r.tag_map_pose = _pose_from_dict(d.get('tag_map_pose'))
    r.robot_map_pose = _pose_from_dict(d.get('robot_map_pose'))
    r.recorded_at = _time_from_dict(d.get('recorded_at', {}))
    r.source_robot = str(d.get('source_robot', ''))
    r.range_at_capture = float(d.get('range_at_capture', 0.0))
    r.room = str(d.get('room', ''))
    return r


def _pose_unset(p: PoseStamped):
    o = p.pose.orientation
    pos = p.pose.position
    if o.x == 0.0 and o.y == 0.0 and o.z == 0.0 and o.w == 0.0:
        return True
    identity = o.x == 0.0 and o.y == 0.0 and o.z == 0.0 and o.w == 1.0
    stamp = p.header.stamp
    return (
        identity
        and pos.x == 0.0 and pos.y == 0.0 and pos.z == 0.0
        and stamp.sec == 0 and stamp.nanosec == 0
    )


def _stamp_str(t: Time):
    return f'{t.sec}.{t.nanosec:09d}'


class QrTagDatabaseNode(Node):
    def __init__(self):
        super().__init__('qr_tag_database')

        self.declare_parameter('database_path', '')
        self.declare_parameter('overwrite_existing', True)

        default_path = os.path.join(
            get_package_share_directory('ms05_auto_mapping'), 'maps', 'qr_tag_database.yaml'
        )
        self._db_path = self.get_parameter('database_path').value or default_path
        self._overwrite = bool(self.get_parameter('overwrite_existing').value)

        self._records = {}
        self._load()
        self.get_logger().info(
            f'qr tag database at {self._db_path} loaded with {len(self._records)} record(s)'
        )

        self.create_service(
            RecordQrTag, 'qr_tag_database/record_qr_tag', self._handle_record
        )
        self.create_service(
            QueryQrTag, 'qr_tag_database/query_qr_tag', self._handle_query
        )

    def _load(self):
        if not os.path.exists(self._db_path):
            return
        try:
            with open(self._db_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            self._records = {
                int(k): _record_from_dict(v) for k, v in data.items()
            }
        except Exception as exc:  # noqa: BLE001 - corrupt/absent file starts empty
            self.get_logger().warning(f'could not load {self._db_path}: {exc}')
            self._records = {}

    def _save(self):
        directory = os.path.dirname(self._db_path) or '.'
        os.makedirs(directory, exist_ok=True)
        payload = {str(k): _record_to_dict(v) for k, v in sorted(self._records.items())}
        fd, tmp = tempfile.mkstemp(prefix='.qr_tag_db_', dir=directory)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                yaml.safe_dump(payload, f, sort_keys=True)
            os.replace(tmp, self._db_path)
        except Exception:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise

    def _handle_record(self, request, response):
        rec = request.record
        if rec.header.frame_id != 'map':
            response.success = False
            response.overwrote_existing = False
            response.message = (
                f"rejected marker {rec.marker_id}: header.frame_id must be "
                f"'map', got '{rec.header.frame_id}'"
            )
            self.get_logger().error(response.message)
            return response
        if rec.marker_id < 0:
            response.success = False
            response.overwrote_existing = False
            response.message = f'rejected marker {rec.marker_id}: marker_id must be >= 0'
            self.get_logger().error(response.message)
            return response
        if _pose_unset(rec.tag_map_pose) or _pose_unset(rec.robot_map_pose):
            response.success = False
            response.overwrote_existing = False
            response.message = (
                f'rejected marker {rec.marker_id}: both tag_map_pose and '
                f'robot_map_pose must be set'
            )
            self.get_logger().error(response.message)
            return response

        if rec.recorded_at.sec == 0 and rec.recorded_at.nanosec == 0:
            rec.recorded_at = self.get_clock().now().to_msg()

        existed = rec.marker_id in self._records
        if existed and not self._overwrite:
            response.success = False
            response.overwrote_existing = False
            response.message = (
                f'rejected marker {rec.marker_id}: already recorded and '
                f'overwrite_existing is false'
            )
            self.get_logger().warning(response.message)
            return response

        original_at = None
        if existed:
            original_at = self._records[rec.marker_id].recorded_at

        self._records[rec.marker_id] = rec
        self._save()

        response.success = True
        response.overwrote_existing = existed
        if existed:
            response.message = (
                f'marker {rec.marker_id} overwritten '
                f'(original recorded_at {_stamp_str(original_at)}, refreshed)'
            )
            self.get_logger().info(response.message)
        else:
            response.message = f'marker {rec.marker_id} recorded'
            self.get_logger().info(response.message)
        return response

    def _handle_query(self, request, response):
        if request.marker_id == -1:
            records = [self._records[k] for k in sorted(self._records)]
            response.success = True
            response.message = f'{len(records)} record(s)'
            response.records = records
            return response
        if request.marker_id in self._records:
            response.success = True
            response.message = '1 record'
            response.records = [self._records[request.marker_id]]
            return response
        response.success = False
        response.message = f'marker {request.marker_id} not found'
        response.records = []
        return response


def main(args=None):
    rclpy.init(args=args)
    node = QrTagDatabaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
