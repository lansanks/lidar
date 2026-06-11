#!/usr/bin/env python3

import copy
import json
import math
import os
import select
import sys
import termios
import threading
import tty
from datetime import datetime, timezone

import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile


DEFAULT_POINTS_DIR = os.path.join(
    os.path.expanduser('~'),
    'lidar',
    'lio',
    'maps',
    'points',
)
DEFAULT_POINTS_FILE = os.path.join(DEFAULT_POINTS_DIR, 'navigation_points.json')
DEFAULT_CHECKS_FILE = os.path.join(DEFAULT_POINTS_DIR, 'point_checks.json')
POINTS_SCHEMA = 'lidar.navigation.points.v1'
CHECKS_SCHEMA = 'lidar.navigation.point_checks.v1'


def now_iso():
    return datetime.now(timezone.utc).isoformat()


def expand_path(path):
    return os.path.abspath(os.path.expanduser(str(path)))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(quaternion):
    x = float(quaternion.get('x', 0.0))
    y = float(quaternion.get('y', 0.0))
    z = float(quaternion.get('z', 0.0))
    w = float(quaternion.get('w', 1.0))

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return None

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_from_odometry(msg):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    pose = {
        'received_at': now_iso(),
        'stamp': {
            'sec': int(msg.header.stamp.sec),
            'nanosec': int(msg.header.stamp.nanosec),
        },
        'frame_id': msg.header.frame_id,
        'child_frame_id': msg.child_frame_id,
        'position': {
            'x': float(position.x),
            'y': float(position.y),
            'z': float(position.z),
        },
        'orientation': {
            'x': float(orientation.x),
            'y': float(orientation.y),
            'z': float(orientation.z),
            'w': float(orientation.w),
        },
    }
    pose['yaw_rad'] = quaternion_yaw(pose['orientation'])
    return pose


def compute_error(reference, measured):
    ref_pos = reference['position']
    measured_pos = measured['position']
    dx = measured_pos['x'] - ref_pos['x']
    dy = measured_pos['y'] - ref_pos['y']
    dz = measured_pos['z'] - ref_pos['z']

    error = {
        'dx': dx,
        'dy': dy,
        'dz': dz,
        'horizontal_error': math.sqrt(dx * dx + dy * dy),
        'error_3d': math.sqrt(dx * dx + dy * dy + dz * dz),
    }

    ref_yaw = reference.get('yaw_rad')
    measured_yaw = measured.get('yaw_rad')
    if ref_yaw is not None and measured_yaw is not None:
        yaw_error = normalize_angle(measured_yaw - ref_yaw)
        error['yaw_error_rad'] = yaw_error
        error['yaw_error_deg'] = math.degrees(yaw_error)

    return error


def navigation_point_from_pose(pose, point_id):
    point = {
        'id': point_id,
        'name': f'point_{point_id:03d}',
        'recorded_at': now_iso(),
        'frame_id': pose.get('frame_id', ''),
        'child_frame_id': pose.get('child_frame_id', ''),
        'stamp': copy.deepcopy(pose.get('stamp', {})),
        'position': copy.deepcopy(pose['position']),
        'orientation': copy.deepcopy(pose['orientation']),
        'yaw_rad': pose.get('yaw_rad'),
    }
    return point


def normalize_navigation_point(point, fallback_id):
    normalized = copy.deepcopy(point)

    if 'pose' in normalized and 'position' not in normalized:
        pose = normalized.pop('pose')
        normalized.update(pose)

    normalized.setdefault('id', fallback_id)
    normalized.setdefault('name', f'point_{int(normalized["id"]):03d}')
    normalized.setdefault('frame_id', '')
    normalized.setdefault('child_frame_id', '')
    normalized.setdefault('stamp', {})
    normalized.setdefault('recorded_at', normalized.get('received_at', now_iso()))

    if 'yaw_rad' not in normalized and 'orientation' in normalized:
        normalized['yaw_rad'] = quaternion_yaw(normalized['orientation'])

    return normalized


class PointsNode(Node):
    def __init__(self):
        super().__init__('points')

        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('points_file', DEFAULT_POINTS_FILE)
        self.declare_parameter('checks_file', DEFAULT_CHECKS_FILE)
        self.declare_parameter('results_file', '')
        self.declare_parameter('mode', 'auto')

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.points_file = expand_path(self.get_parameter('points_file').value)
        checks_file = str(self.get_parameter('checks_file').value).strip()
        results_file = str(self.get_parameter('results_file').value).strip()
        self.checks_file = expand_path(results_file or checks_file)
        self.mode = str(self.get_parameter('mode').value).strip().lower()

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._shutdown_requested = False
        self._saved_on_exit = False
        self._latest_pose = None
        self._references = []
        self._results = []
        self._next_check_index = 0

        self._load_references()
        self._resolve_mode()

        qos = QoSProfile(depth=20)
        self._subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            qos,
        )

        self._keyboard_thread = threading.Thread(
            target=self._keyboard_loop,
            name='points_keyboard',
            daemon=True,
        )
        self._keyboard_thread.start()

        self.get_logger().info(f'Subscribed to odometry topic: {self.odom_topic}')
        self.get_logger().info(f'Points file: {self.points_file}')
        self.get_logger().info(f'Checks file: {self.checks_file}')
        self._print_help()

    def _resolve_mode(self):
        valid_modes = {'auto', 'record', 'check'}
        if self.mode not in valid_modes:
            self.get_logger().warn(
                f'Unknown mode "{self.mode}". Falling back to auto mode.'
            )
            self.mode = 'auto'

        if self.mode == 'auto':
            self.mode = 'check' if self._references else 'record'

    def _odom_callback(self, msg):
        pose = pose_from_odometry(msg)
        with self._lock:
            self._latest_pose = pose

    def _latest_pose_snapshot(self):
        with self._lock:
            if self._latest_pose is None:
                return None
            return copy.deepcopy(self._latest_pose)

    def _load_references(self):
        if not os.path.exists(self.points_file):
            return

        try:
            with open(self.points_file, 'r', encoding='utf-8') as file:
                data = json.load(file)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().error(f'Failed to load points file: {exc}')
            return

        points = data.get('points', [])
        if not isinstance(points, list):
            self.get_logger().error('Invalid points file: "points" must be a list.')
            return

        points = [
            normalize_navigation_point(point, index + 1)
            for index, point in enumerate(points)
        ]

        self._references = points
        self.get_logger().info(f'Loaded {len(self._references)} reference point(s).')

    def _save_references(self):
        with self._lock:
            references = copy.deepcopy(self._references)

        os.makedirs(os.path.dirname(os.path.abspath(self.points_file)), exist_ok=True)
        data = {
            'schema': POINTS_SCHEMA,
            'version': 1,
            'kind': 'navigation_points',
            'updated_at': now_iso(),
            'odom_topic': self.odom_topic,
            'frame_id': self._points_frame_id(references),
            'points': references,
        }
        with open(self.points_file, 'w', encoding='utf-8') as file:
            json.dump(data, file, indent=2)
            file.write('\n')
        self.get_logger().info(f'Saved {len(references)} point(s).')

    def _save_results(self):
        with self._lock:
            results = copy.deepcopy(self._results)

        if not results:
            return

        os.makedirs(os.path.dirname(os.path.abspath(self.checks_file)), exist_ok=True)
        data = {
            'schema': CHECKS_SCHEMA,
            'version': 1,
            'kind': 'point_checks',
            'updated_at': now_iso(),
            'odom_topic': self.odom_topic,
            'points_file': self.points_file,
            'checks': results,
        }
        with open(self.checks_file, 'w', encoding='utf-8') as file:
            json.dump(data, file, indent=2)
            file.write('\n')
        self.get_logger().info(f'Saved {len(results)} check result(s).')

    def _points_frame_id(self, references):
        frame_ids = {
            point.get('frame_id')
            for point in references
            if point.get('frame_id')
        }
        if len(frame_ids) == 1:
            return next(iter(frame_ids))
        if not frame_ids:
            return ''
        return 'mixed'

    def _record_point(self):
        pose = self._latest_pose_snapshot()
        if pose is None:
            self.get_logger().warn('No odometry received yet. Cannot record point.')
            return

        with self._lock:
            point_id = len(self._references) + 1
            point = navigation_point_from_pose(pose, point_id)
            self._references.append(point)

        self.get_logger().info(
            f'Recorded point {point_id}: {self._format_pose(point)}'
        )
        self._save_references()

    def _undo_last_reference(self):
        with self._lock:
            if not self._references:
                self.get_logger().warn('No reference point to undo.')
                return
            point = self._references.pop()

        self.get_logger().info(
            f'Removed point {point.get("id", len(self._references) + 1)}.'
        )
        self._save_references()

    def _compare_next_reference(self):
        with self._lock:
            index = self._next_check_index

        self._compare_reference(index)

    def _compare_reference(self, index):
        pose = self._latest_pose_snapshot()
        if pose is None:
            self.get_logger().warn('No odometry received yet. Cannot compare point.')
            return

        with self._lock:
            if not self._references:
                self.get_logger().warn('No reference points loaded.')
                return
            if index < 0 or index >= len(self._references):
                self.get_logger().warn(
                    f'Point index {index + 1} is out of range. '
                    f'There are {len(self._references)} reference point(s).'
                )
                return
            reference = copy.deepcopy(self._references[index])

        error = compute_error(reference, pose)
        result = {
            'point_index': index,
            'point_id': reference.get('id', index + 1),
            'checked_at': now_iso(),
            'reference': reference,
            'measured': pose,
            'error': error,
        }

        with self._lock:
            existing = next(
                (
                    i for i, item in enumerate(self._results)
                    if item.get('point_index') == index
                ),
                None,
            )
            if existing is None:
                self._results.append(result)
            else:
                self._results[existing] = result
            self._advance_next_check_index_locked()
            complete = len({item['point_index'] for item in self._results}) == len(
                self._references
            )

        self._print_result(result)
        self._save_results()
        if complete:
            self._print_summary()

    def _advance_next_check_index_locked(self):
        checked = {item.get('point_index') for item in self._results}
        self._next_check_index = 0
        while (
            self._next_check_index < len(self._references)
            and self._next_check_index in checked
        ):
            self._next_check_index += 1

    def _print_status(self):
        pose = self._latest_pose_snapshot()
        with self._lock:
            reference_count = len(self._references)
            result_count = len({item.get('point_index') for item in self._results})
            next_index = self._next_check_index + 1

        self.get_logger().info(
            f'Mode: {self.mode}, references: {reference_count}, '
            f'checked: {result_count}/{reference_count}'
        )
        if pose is None:
            self.get_logger().info('Current pose: waiting for odometry...')
        else:
            self.get_logger().info(f'Current pose: {self._format_pose(pose)}')
        if self.mode == 'check' and next_index <= reference_count:
            self.get_logger().info(f'Next reference point: {next_index}')

    def _print_help(self):
        self.get_logger().info(
            'Keys: r=record point, c/space/enter=check next point, '
            '1-9=check point by index, u=undo last recorded point, '
            'p=print status/summary, s=save, q=save and quit, h=help'
        )
        self._print_status()

    def _print_result(self, result):
        error = result['error']
        message = (
            f'Point {result["point_index"] + 1}: '
            f'dx={error["dx"]:.4f} m, dy={error["dy"]:.4f} m, '
            f'dz={error["dz"]:.4f} m, horizontal={error["horizontal_error"]:.4f} m, '
            f'3d={error["error_3d"]:.4f} m'
        )
        if 'yaw_error_deg' in error:
            message += f', yaw={error["yaw_error_deg"]:.3f} deg'
        self.get_logger().info(message)

    def _print_summary(self):
        with self._lock:
            results = copy.deepcopy(self._results)

        if not results:
            self.get_logger().info('No localization errors recorded yet.')
            return

        results.sort(key=lambda item: item['point_index'])
        horizontal_errors = [item['error']['horizontal_error'] for item in results]
        errors_3d = [item['error']['error_3d'] for item in results]

        self.get_logger().info('Localization error summary:')
        for result in results:
            self._print_result(result)

        self.get_logger().info(
            f'Horizontal error: avg={sum(horizontal_errors) / len(horizontal_errors):.4f} m, '
            f'max={max(horizontal_errors):.4f} m'
        )
        self.get_logger().info(
            f'3D error: avg={sum(errors_3d) / len(errors_3d):.4f} m, '
            f'max={max(errors_3d):.4f} m'
        )

    def _format_pose(self, pose):
        pos = pose['position']
        text = f'x={pos["x"]:.4f}, y={pos["y"]:.4f}, z={pos["z"]:.4f}'
        yaw = pose.get('yaw_rad')
        if yaw is not None:
            text += f', yaw={math.degrees(yaw):.3f} deg'
        frame_id = pose.get('frame_id')
        if frame_id:
            text += f', frame={frame_id}'
        return text

    def _handle_key(self, key):
        if key in {'\x03', 'q'}:
            self.get_logger().info('Saving before shutdown.')
            self._shutdown_requested = True
            self.close()
            return
        if key in {'h', '?'}:
            self._print_help()
            return
        if key == 'r':
            self._record_point()
            return
        if key == 'u':
            self._undo_last_reference()
            return
        if key in {'c', ' ', '\n', '\r'}:
            self._compare_next_reference()
            return
        if key in {'p'}:
            self._print_status()
            self._print_summary()
            return
        if key == 's':
            self._save_references()
            self._save_results()
            return
        if key in '123456789':
            self._compare_reference(int(key) - 1)
            return

    def _keyboard_loop(self):
        if sys.stdin.isatty():
            self._tty_keyboard_loop()
        else:
            self._line_keyboard_loop()

    def _tty_keyboard_loop(self):
        file_descriptor = sys.stdin.fileno()
        old_settings = termios.tcgetattr(file_descriptor)
        try:
            tty.setcbreak(file_descriptor)
            while not self._stop_event.is_set() and rclpy.ok():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if readable:
                    self._handle_key(sys.stdin.read(1))
        finally:
            termios.tcsetattr(file_descriptor, termios.TCSADRAIN, old_settings)

    def _line_keyboard_loop(self):
        while not self._stop_event.is_set() and rclpy.ok():
            line = sys.stdin.readline()
            if not line:
                break
            self._handle_key(line[0])

    def close(self):
        if self._saved_on_exit:
            return

        self._saved_on_exit = True
        self._stop_event.set()
        with self._lock:
            has_references = bool(self._references)
            has_results = bool(self._results)

        if has_references:
            self._save_references()
        if has_results:
            self._print_summary()
            self._save_results()

        if (
            hasattr(self, '_keyboard_thread')
            and threading.current_thread() is not self._keyboard_thread
            and self._keyboard_thread.is_alive()
        ):
            self._keyboard_thread.join(timeout=0.5)

    @property
    def shutdown_requested(self):
        return self._shutdown_requested


def main(args=None):
    rclpy.init(args=args)
    node = PointsNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.shutdown_requested:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        executor.remove_node(node)
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
