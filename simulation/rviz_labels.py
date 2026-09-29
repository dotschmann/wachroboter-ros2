#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


MISSION_POINTS = [
    (0, 'GUARD_POSITION', -1.185, -0.100),
    (1, 'VISITOR_STOP', -1.850, 1.442),
    (2, 'GUARD_CLEAR', -0.349, -0.731),
    (3, 'LAGER', 0.343, 1.442),
    (4, 'POLIZEI', -2.001, 6.053),
]


class MissionLabels(Node):
    def __init__(self):
        super().__init__('mission_labels')

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(
            MarkerArray,
            '/wachroboter/labels',
            qos,
        )
        self.timer = self.create_timer(1.0, self.publish_labels)

    def make_label(self, marker_id, name, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'wachroboter_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.35
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = name
        return marker

    def publish_labels(self):
        markers = MarkerArray()
        for marker_id, name, x, y in MISSION_POINTS:
            markers.markers.append(self.make_label(marker_id, name, x, y))
        self.pub.publish(markers)


def main():
    rclpy.init()
    node = MissionLabels()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
