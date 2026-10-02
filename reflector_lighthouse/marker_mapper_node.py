#!/usr/bin/env python3
"""
Auto-collect reflective markers into a YAML config while the robot moves.

Services:
  /marker_mapper/save  — save all markers to YAML
  /marker_mapper/clear — clear markers
"""

import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from reflector_lighthouse.common import (
    find_stars, stars_to_map, yaw_from_quat
)


@dataclass
class MarkerTrack:
    x: float
    y: float
    count: int = 1

    def __post_init__(self) -> None:
        self.hits = [(self.x, self.y)]

    def update(self, x: float, y: float) -> None:
        self.hits.append((x, y))
        self.count = len(self.hits)
        # take average coords
        self.x = sum(h[0] for h in self.hits) / len(self.hits)
        self.y = sum(h[1] for h in self.hits) / len(self.hits)


class MarkerMapperNode(Node):
    def __init__(self) -> None:
        super().__init__('marker_mapper_node')

        self.declare_parameter('scan_topic', '/scan_light')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('intensity_min', 200.0)
        self.declare_parameter('cluster_radius_m', 0.03)
        self.declare_parameter('merge_radius_m', 0.25)
        self.declare_parameter('min_observations', 3)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('lidar_x', 0.6)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_yaw', -3.14)
        default_out = str(
            Path(get_package_share_directory('reflector_lighthouse'))
            / 'config'
            / 'markers.yaml'
        )
        self.declare_parameter('output_file', default_out)
        self.declare_parameter('enabled', True)
        self.declare_parameter('save_on_shutdown', True)

        self._markers: List[MarkerTrack] = []
        self._pose: Optional[Tuple[float, float, float]] = None
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._output_file = Path(
            str(self.get_parameter('output_file').value)
        ).expanduser()
        self._shutdown_saved = False

        self._markers_pub = self.create_publisher(
            MarkerArray, 'marker_mapper/markers', 10
        )
        self._raw_pub = self.create_publisher(
            MarkerArray, 'marker_mapper/detections', 10
            )

        self.create_subscription(
            Odometry, str(
                self.get_parameter('odom_topic').value),
                self._on_odom, 20
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_service(
            Trigger, '/marker_mapper/save', self._on_save
        )
        self.create_service(
            Trigger, '/marker_mapper/clear', self._on_clear
        )
        self.create_subscription(
            Empty, '/marker_mapper/save', self._on_save_topic, 10
        )
        self.create_timer(0.5, self._publish_markers)

        self.get_logger().info(f'output_file = {self._output_file}')

    def _on_odom(self, msg: Odometry) -> None:
        """Save pose of robot."""
        self._pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quat(msg.pose.pose.orientation),
        )

    def _on_scan(self, msg: LaserScan) -> None:
        """Find stars."""
        if not bool(self.get_parameter('enabled').value) or self._pose is None:
            return

        if msg.intensities:
            intens = list(msg.intensities)
        else:
            intens = [255.0] * len(msg.ranges)
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
            self._validate(x, y, merge_r)

    def _validate(self, x: float, y: float, merge_r: float) -> None:
        """Merge detection into nearest track or start a new one."""
        best = min(
            self._markers,
            key=lambda t: math.hypot(t.x - x, t.y - y),
            default=None,
        )
        if best is None or math.hypot(best.x - x, best.y - y) >= merge_r:
            self._markers.append(MarkerTrack(x, y))
            self.get_logger().info(
                f'New track #{len(self._markers)}: ({x:.3f}, {y:.3f})'
            )
        else:
            best.update(x, y)

    def _write_yaml(self, markers: List[MarkerTrack]) -> Tuple[bool, str]:
        """Make yaml and write."""
        if not markers:
            return False, f'Nothing to save (markers={len(self._markers)})'

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
                    for i, t in enumerate(markers)
                ]
            }
            text = yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
            path.write_text(text, encoding='utf-8')
            if not path.is_file() or path.stat().st_size == 0:
                return False, f'Write failed (empty/missing): {path}'

            self.get_logger().info(
                f'SAVED {len(markers)} marker(s) -> {path.resolve()}'
            )
            return True, f'Saved {len(markers)} markers to {path.resolve()}'
        except Exception as e:  # noqa: BLE001
            err = f'Failed to write {path}: {e}'
            self.get_logger().error(err)
            return False, err

    def _on_save(self, _req, res):
        """Save markers."""
        markers = list(self._markers)
        if not markers:
            res.success = False
            res.message = 'Nothing to save (no markers)'
            self.get_logger().warn(res.message)
            return res
        self.get_logger().info(f'saving {len(markers)} markers')
        ok, msg = self._write_yaml(markers)
        res.success = ok
        res.message = msg
        return res

    def _on_save_topic(self, _msg) -> None:
        """Response to trigger."""
        res = self._on_save(None, Trigger.Response())
        self.get_logger().info(f'success={res.success} {res.message}')

    def _on_clear(self, _req, res):
        """Clean markers."""
        n = len(self._markers)
        self._markers.clear()
        res.success = True
        res.message = f'Cleared {n} markers'
        self.get_logger().info(res.message)
        return res

    def _publish_raw(self, xy, stamp) -> None:
        """Publish reflects by lidar now."""
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

    def _publish_markers(self) -> None:
        """Pub markers as dots to ros2."""
        stamp = self.get_clock().now().to_msg()
        need = int(self.get_parameter('min_observations').value)
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self._map_frame
        clear.header.stamp = stamp
        clear.ns = 'markers'
        clear.id = 0
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, t in enumerate(self._markers):
            ready = t.count >= need
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = stamp
            m.ns = 'markers'
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

        accepted = list(self._markers)
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
        markers = list(self._markers)
        if not markers:
            self.get_logger().warn('Shutdown: no markers to save')
            return
        self.get_logger().info(
            f'Shutdown: auto-saving {len(markers)} marker(s)...'
        )
        ok, msg = self._write_yaml(markers)
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
