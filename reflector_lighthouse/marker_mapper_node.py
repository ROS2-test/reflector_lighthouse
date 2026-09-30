#!/usr/bin/env python3
"""
Auto-collect reflective markers into a YAML config while the robot moves.

Services:
  /marker_mapper_node/save_markers       — save markers meeting min_observations
  /marker_mapper_node/save_markers_force — save all tracks (even 1 hit)
  /marker_mapper_node/clear_markers      — clear tracks
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from reflector_lighthouse.stars import find_stars, stars_to_map, yaw_from_quat


@dataclass
class MarkerTrack:
    x: float
    y: float
    count: int = 1
    hits: List[Tuple[float, float]] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.hits:
            self.hits = [(self.x, self.y)]

    def update(self, x: float, y: float, max_hits: int = 50) -> None:
        self.hits.append((x, y))
        if len(self.hits) > max_hits:
            self.hits = self.hits[-max_hits:]
        self.count = len(self.hits)
        self.x = sum(h[0] for h in self.hits) / len(self.hits)
        self.y = sum(h[1] for h in self.hits) / len(self.hits)


class MarkerMapperNode(Node):
    def __init__(self) -> None:
        super().__init__('marker_mapper_node')

        self.declare_parameter('scan_topic', '/scan_light')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('intensity_min', 190.0)
        self.declare_parameter('cluster_radius_m', 0.07)
        self.declare_parameter('merge_radius_m', 0.25)
        self.declare_parameter('min_observations', 3)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('lidar_x', 0.01)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_yaw', 3.14)
        default_out = str(
            Path.home() / 'ros2_ws/src/reflector_lighthouse/config/markers.yaml'
        )
        self.declare_parameter('output_file', default_out)
        self.declare_parameter('enabled', True)
        self.declare_parameter('save_on_shutdown', True)

        self._tracks: List[MarkerTrack] = []
        self._pose: Optional[Tuple[float, float, float]] = None
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._output_file = Path(str(self.get_parameter('output_file').value)).expanduser()
        self._last_ready_count = 0
        self._last_status_log = 0.0
        self._shutdown_saved = False

        self._markers_pub = self.create_publisher(MarkerArray, 'marker_mapper/markers', 10)
        self._raw_pub = self.create_publisher(MarkerArray, 'marker_mapper/detections', 10)

        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 20
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_service(Trigger, '/marker_mapper/save', self._on_save)
        self.create_service(Trigger, '/marker_mapper/save_force', self._on_save_force)
        self.create_service(Trigger, '/marker_mapper/clear', self._on_clear)
        self.create_subscription(Empty, '/marker_mapper/save', self._on_save_topic, 10)
        self.create_subscription(Empty, '/marker_mapper/save_force', self._on_save_force_topic, 10)
        self.create_timer(0.5, self._publish_tracks)
        self.create_timer(2.0, self._log_status)

        self.get_logger().info(f'output_file = {self._output_file}')
        self.get_logger().info(
            'Save commands (note: Trigger, NOT Triggerer):\n'
            '  ros2 service call /marker_mapper/save std_srvs/srv/Trigger\n'
            '  ros2 service call /marker_mapper/save_force std_srvs/srv/Trigger\n'
            'Or:\n'
            '  ros2 topic pub --once /marker_mapper/save std_msgs/msg/Empty "{}"\n'
            '  ros2 topic pub --once /marker_mapper/save_force std_msgs/msg/Empty "{}"'
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quat(msg.pose.pose.orientation),
        )

    def _on_scan(self, msg: LaserScan) -> None:
        if not bool(self.get_parameter('enabled').value) or self._pose is None:
            return

        intens = list(msg.intensities) if msg.intensities else [255.0] * len(msg.ranges)
        if len(intens) < len(msg.ranges):
            intens += [0.0] * (len(msg.ranges) - len(intens))

        stars = find_stars(
            msg.ranges,
            intens,
            msg.angle_min,
            msg.angle_increment,
            float(self.get_parameter('intensity_min').value),
            float(self.get_parameter('cluster_radius_m').value),
        )
        max_range = float(self.get_parameter('max_range_m').value)
        stars = [(d, a) for d, a in stars if d <= max_range]

        px, py, yaw = self._pose
        xy = stars_to_map(
            stars,
            px,
            py,
            yaw,
            float(self.get_parameter('lidar_x').value),
            float(self.get_parameter('lidar_y').value),
            float(self.get_parameter('lidar_yaw').value),
        )
        self._publish_raw(xy, msg.header.stamp)

        merge_r = float(self.get_parameter('merge_radius_m').value)
        for x, y in xy:
            self._ingest(x, y, merge_r)
        self._check_ready_logs()

    def _ingest(self, x: float, y: float, merge_r: float) -> None:
        best = None
        best_d = merge_r
        for t in self._tracks:
            d = math.hypot(t.x - x, t.y - y)
            if d < best_d:
                best_d = d
                best = t
        if best is None:
            self._tracks.append(MarkerTrack(x, y))
            n = len(self._tracks)
            self.get_logger().info(
                f'New marker track #{n}: x={x:.3f} y={y:.3f} '
                f'(total tracks={n})'
            )
        else:
            best.update(x, y)

    def _check_ready_logs(self) -> None:
        ready = len(self._accepted(force=False))
        if ready > self._last_ready_count:
            for k in range(self._last_ready_count + 1, ready + 1):
                self.get_logger().info(f'Ready marker {k} (enough observations)')
            self.get_logger().info(
                f'Ready markers: {ready} / tracks: {len(self._tracks)} '
                f'(min_observations={int(self.get_parameter("min_observations").value)})'
            )
            self._last_ready_count = ready

    def _log_status(self) -> None:
        if self._pose is None:
            self.get_logger().warn(
                'No odometry yet on '
                f'{self.get_parameter("odom_topic").value} — tracks will stay empty'
            )
            return
        need = int(self.get_parameter('min_observations').value)
        ready = len(self._accepted(force=False))
        parts = [
            f'#{i}: hits={t.count}/{need} ({t.x:.2f},{t.y:.2f})'
            for i, t in enumerate(self._tracks)
        ]
        summary = ', '.join(parts) if parts else 'none'
        self.get_logger().info(
            f'Status: ready={ready}, tracks={len(self._tracks)} | {summary}'
        )

    def _accepted(self, force: bool = False) -> List[MarkerTrack]:
        if force:
            return list(self._tracks)
        need = int(self.get_parameter('min_observations').value)
        return [t for t in self._tracks if t.count >= need]

    def _write_yaml(self, tracks: List[MarkerTrack]) -> Tuple[bool, str]:
        if not tracks:
            return False, f'Nothing to save (tracks={len(self._tracks)})'

        path = self._output_file
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            data = {
                'markers': [
                    {
                        'id': i,
                        'x': round(t.x, 4),
                        'y': round(t.y, 4),
                        'observations': int(t.count),
                    }
                    for i, t in enumerate(tracks)
                ]
            }
            text = yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
            path.write_text(text, encoding='utf-8')
            if not path.is_file() or path.stat().st_size == 0:
                return False, f'Write failed (empty/missing): {path}'

            self.get_logger().info('=' * 50)
            self.get_logger().info(
                f'SAVED {len(tracks)} marker(s) -> {path.resolve()}'
            )
            for i, t in enumerate(tracks):
                self.get_logger().info(
                    f'  marker {i + 1}/{len(tracks)}: '
                    f'id={i} x={t.x:.4f} y={t.y:.4f} hits={t.count}'
                )
            self.get_logger().info('=' * 50)
            msg = f'Saved {len(tracks)} markers to {path.resolve()}'
            return True, msg
        except Exception as exc:  # noqa: BLE001
            err = f'Failed to write {path}: {exc}'
            self.get_logger().error(err)
            return False, err

    def _on_save(self, _req, res):
        accepted = self._accepted(force=False)
        need = int(self.get_parameter('min_observations').value)
        if not accepted:
            res.success = False
            res.message = (
                f'No markers with >= {need} hits '
                f'(tracks={len(self._tracks)}). '
                f'Use /marker_mapper/save_force or wait longer.'
            )
            self.get_logger().warn(res.message)
            return res
        self.get_logger().info(f'save: saving {len(accepted)} ready marker(s)...')
        ok, msg = self._write_yaml(accepted)
        res.success = ok
        res.message = msg
        return res

    def _on_save_force(self, _req, res):
        tracks = self._accepted(force=True)
        self.get_logger().info(f'save_force: saving {len(tracks)} track(s)...')
        ok, msg = self._write_yaml(tracks)
        res.success = ok
        res.message = msg
        return res

    def _on_save_topic(self, _msg) -> None:
        class _Res:
            success = False
            message = ''
        res = _Res()
        self._on_save(None, res)
        self.get_logger().info(f'topic save -> success={res.success} {res.message}')

    def _on_save_force_topic(self, _msg) -> None:
        class _Res:
            success = False
            message = ''
        res = _Res()
        self._on_save_force(None, res)
        self.get_logger().info(f'topic save_force -> success={res.success} {res.message}')

    def _on_clear(self, _req, res):
        n = len(self._tracks)
        self._tracks.clear()
        self._last_ready_count = 0
        res.success = True
        res.message = f'Cleared {n} tracks'
        self.get_logger().info(res.message)
        return res

    def _publish_raw(self, xy, stamp) -> None:
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self._map_frame
        clear.header.stamp = stamp
        clear.ns = 'detections'
        clear.id = 0
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (x, y) in enumerate(xy):
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = stamp
            m.ns = 'detections'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.0, 0.8
            arr.markers.append(m)
        self._raw_pub.publish(arr)

    def _publish_tracks(self) -> None:
        stamp = self.get_clock().now().to_msg()
        need = int(self.get_parameter('min_observations').value)
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self._map_frame
        clear.header.stamp = stamp
        clear.ns = 'tracks'
        clear.id = 0
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, t in enumerate(self._tracks):
            ready = t.count >= need
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = stamp
            m.ns = 'tracks'
            m.id = i + 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = t.x
            m.pose.position.y = t.y
            m.pose.position.z = 0.4
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.15
            m.scale.z = 0.8
            if ready:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 1.0, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.6, 0.6, 0.6, 0.5
            arr.markers.append(m)

            text = Marker()
            text.header = m.header
            text.ns = 'labels'
            text.id = i + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = t.x
            text.pose.position.y = t.y
            text.pose.position.z = 1.0
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f'{i}:{t.count}'
            arr.markers.append(text)

        accepted = self._accepted(force=False)
        if len(accepted) >= 2:
            lines = Marker()
            lines.header.frame_id = self._map_frame
            lines.header.stamp = stamp
            lines.ns = 'edges'
            lines.id = 1
            lines.type = Marker.LINE_LIST
            lines.action = Marker.ADD
            lines.scale.x = 0.02
            lines.color.r, lines.color.g, lines.color.b, lines.color.a = 0.2, 1.0, 0.4, 0.6
            lines.pose.orientation.w = 1.0
            for i in range(len(accepted)):
                for j in range(i + 1, len(accepted)):
                    lines.points.append(Point(x=accepted[i].x, y=accepted[i].y, z=0.05))
                    lines.points.append(Point(x=accepted[j].x, y=accepted[j].y, z=0.05))
            arr.markers.append(lines)

        self._markers_pub.publish(arr)

    def save_on_shutdown(self) -> None:
        """Auto-save when node stops (Ctrl+C / launch shutdown)."""
        if self._shutdown_saved:
            return
        self._shutdown_saved = True
        if not bool(self.get_parameter('save_on_shutdown').value):
            self.get_logger().info('save_on_shutdown=false, skip auto-save')
            return
        ready = self._accepted(force=False)
        tracks = ready if ready else self._accepted(force=True)
        if not tracks:
            self.get_logger().warn('Shutdown: no markers to save')
            return
        self.get_logger().info(
            f'Shutdown: auto-saving {len(tracks)} marker(s) '
            f'({"ready" if ready else "all tracks"})...'
        )
        ok, msg = self._write_yaml(tracks)
        if ok:
            self.get_logger().info(f'Shutdown save OK: {msg}')
        else:
            self.get_logger().error(f'Shutdown save failed: {msg}')

    def destroy_node(self) -> bool:
        try:
            self.save_on_shutdown()
        except Exception as exc:  # noqa: BLE001
            try:
                self.get_logger().error(f'Shutdown save exception: {exc}')
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MarkerMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save_on_shutdown()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
