#!/usr/bin/env python3
"""
Publish PoseWithCovarianceStamped (/light_pose) for map-EKF from reflective markers.

Inputs:  /scan_light, /odometry/filtered, markers YAML
Output:  /light_pose  (geometry_msgs/PoseWithCovarianceStamped, frame=map)
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from reflector_lighthouse.common import (
    read_pairs,
    build_pairs,
    find_stars,
    quat_from_yaw,
    stars_to_map,
    yaw_from_quat,
)


def estimate_pose(
    stars_xy: List[Tuple[float, float]],
    pairs,
    px: float,
    py: float,
    yaw: float,
    max_dist_err: float = 0.08,
    max_xy_err: float = 0.5,
) -> Optional[Tuple[float, float, float, int]]:
    if len(stars_xy) < 2 or not pairs:
        return None

    c, s = math.cos(yaw), math.sin(yaw)
    sum_dx = sum_dy = sum_c = sum_s = 0.0
    votes = 0
    max_sin = math.sin(math.radians(25.0))

    for i, (sx0, sy0) in enumerate(stars_xy):
        for j, (sx1, sy1) in enumerate(stars_xy):
            if i == j:
                continue
            d_obs = math.hypot(sx1 - sx0, sy1 - sy0)

            lo, hi = 0, len(pairs) - 1
            target = d_obs - max_dist_err
            while lo <= hi:
                mid = (lo + hi) // 2
                if pairs[mid][0] < target:
                    lo = mid + 1
                else:
                    hi = mid - 1
            z = lo

            while z < len(pairs) and abs(d_obs - pairs[z][0]) < max_dist_err:
                _, lx0, ly0, lx1, ly1 = pairs[z]
                lx, ly = lx1 - lx0, ly1 - ly0
                ox, oy = sx1 - sx0, sy1 - sy0
                zn = math.hypot(lx, ly) * math.hypot(ox, oy)
                if zn <= 0.0:
                    z += 1
                    continue
                cos_fi = max(-1.0, min(1.0, (lx * ox + ly * oy) / zn))
                sin_fi = math.sqrt(max(0.0, 1.0 - cos_fi * cos_fi))
                if (lx * oy - ly * ox) < 0.0:
                    sin_fi = -sin_fi
                sin_fi = -sin_fi

                rx = (sx0 - px) * cos_fi - (sy0 - py) * sin_fi
                ry = (sx0 - px) * sin_fi + (sy0 - py) * cos_fi
                dx = lx0 - (rx + px)
                dy = ly0 - (ry + py)

                if (
                    abs(dx) < max_xy_err
                    and abs(dy) < max_xy_err
                    and cos_fi > 0.9
                    and abs(sin_fi) < max_sin
                ):
                    sum_dx += dx
                    sum_dy += dy
                    sum_c += cos_fi
                    sum_s += sin_fi
                    votes += 1
                z += 1

    if votes == 0:
        return None

    dx = sum_dx / votes
    dy = sum_dy / votes
    n = math.hypot(sum_c, sum_s)
    if n <= 0.0:
        return None
    cos_fi, sin_fi = sum_c / n, sum_s / n
    new_c = c * cos_fi - s * sin_fi
    new_s = s * cos_fi + c * sin_fi
    return px + dx, py + dy, math.atan2(new_s, new_c), votes


class LighthouseNode(Node):
    def __init__(self) -> None:
        super().__init__('lighthouse_node')

        self.declare_parameter('scan_topic', '/scan_light')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('pose_topic', '/light_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('markers_file', '')
        self.declare_parameter('intensity_min', 200.0)
        self.declare_parameter('cluster_radius_m', 0.03)
        self.declare_parameter('max_dist_error', 0.08)
        self.declare_parameter('max_xy_error', 2.0)
        self.declare_parameter('max_pair_dist', 30.0)
        self.declare_parameter('min_votes', 2)
        self.declare_parameter('lidar_x', 0.01)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_yaw', 3.14)
        self.declare_parameter('pose_variance_xy', 0.05)
        self.declare_parameter('pose_variance_yaw', 0.05)
        self.declare_parameter('max_w', 0.3)

        self._pairs = []
        self._markers: List[Tuple[float, float]] = []
        self._pose: Optional[Tuple[float, float, float]] = None
        self._w = 0.0
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._load_markers()

        pose_topic = str(self.get_parameter('pose_topic').value)
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, pose_topic, 10
        )
        self._map_markers_pub = self.create_publisher(
            MarkerArray, 'lighthouse/map_markers', 10
        )
        self._stars_pub = self.create_publisher(
            MarkerArray, 'lighthouse/stars', 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom, 20
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._publish_map_markers)
        self.get_logger().info(
            f'light_pose publisher: pairs={len(self._pairs)} -> {pose_topic}'
        )

    def _load_markers(self) -> None:
        """Build or read pairs of markers."""
        path = str(self.get_parameter('markers_file').value)
        if not path:
            self.get_logger().warn('markers_file empty')
            return
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}

        # parse markers
        markers = []
        for m in data.get('markers', []):
            if isinstance(m, dict):
                markers.append((float(m['x']), float(m['y'])))
            else:
                markers.append((float(m[0]), float(m[1])))

        # parse collections or ubild it
        self._markers = markers
        collections = data.get('constellations', [])
        if collections == []:
            self._pairs = build_pairs(
                markers,
                float(self.get_parameter('max_pair_dist').value)
            )
        else:
            self._pairs = read_pairs(self._markers, collections)
        self.get_logger().info(f'Loaded {len(markers)} markers')

    def _publish_map_markers(self) -> None:
        """Pub markers as dots to ros2."""
        if not self._markers:
            return
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self._map_frame
        clear.header.stamp = stamp
        clear.ns = 'map_markers'
        clear.id = 0
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (x, y) in enumerate(self._markers):
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = stamp
            m.ns = 'map_markers'
            m.id = i + 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.15
            m.scale.z = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.85, 1.0, 0.9
            arr.markers.append(m)

            text = Marker()
            text.header = m.header
            text.ns = 'map_labels'
            text.id = i + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 1.2
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f'M{i}'
            arr.markers.append(text)
        self._map_markers_pub.publish(arr)

    def _publish_stars(
        self, stars_xy: List[Tuple[float, float]], stamp
    ) -> None:
        """Publish reflects by lidar now."""
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self._map_frame
        clear.header.stamp = stamp
        clear.ns = 'stars'
        clear.id = 0
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (x, y) in enumerate(stars_xy):
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = stamp
            m.ns = 'stars'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.1, 0.95
            arr.markers.append(m)
        self._stars_pub.publish(arr)

    def _on_odom(self, msg: Odometry) -> None:
        self._pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quat(msg.pose.pose.orientation),
        )
        self._yaw_rate = float(msg.twist.twist.angular.z)

    def _on_scan(self, msg: LaserScan) -> None:
        if self._pose is None or not self._pairs:
            return

        intens = msg.intensities

        stars = find_stars(
            msg.ranges,
            intens,
            msg.angle_min,
            msg.angle_increment,
            float(self.get_parameter('intensity_min').value),
            float(self.get_parameter('cluster_radius_m').value),
        )
        px, py, yaw = self._pose
        stars_xy = stars_to_map(
            stars,
            px,
            py,
            yaw,
            float(self.get_parameter('lidar_x').value),
            float(self.get_parameter('lidar_y').value),
            float(self.get_parameter('lidar_yaw').value),
        )
        self._publish_stars(stars_xy, msg.header.stamp)
        if len(stars) < 2:
            return
        # if w twist > max_w dont pub pose
        if abs(self._w) > float(self.get_parameter('max_w').value):
            return
        result = estimate_pose(
            stars_xy,
            self._pairs,
            px,
            py,
            yaw,
            max_dist_err=float(self.get_parameter('max_dist_error').value),
            max_xy_err=float(self.get_parameter('max_xy_error').value),
        )
        if result is None:
            return
        x, y, yaw_out, votes = result
        if votes < int(self.get_parameter('min_votes').value):
            return

        qx, qy, qz, qw = quat_from_yaw(yaw_out)
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._map_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        cov = [0.0] * 36
        var_xy = float(self.get_parameter('pose_variance_xy').value)
        var_yaw = float(self.get_parameter('pose_variance_yaw').value)
        # Slightly inflate covariance when few votes
        scale = 1.0 + 2.0 / max(votes, 1)
        cov[0] = var_xy * scale
        cov[7] = var_xy * scale
        cov[35] = var_yaw * scale
        out.pose.covariance = cov
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LighthouseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
