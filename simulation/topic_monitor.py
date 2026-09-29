#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class WachroboterMonitor(Node):

    def __init__(self):
        super().__init__('wachroboter_monitor')

        self.create_subscription(
            String,
            '/wachroboter/guard_status',
            self.guard_status_cb,
            10
        )

        self.create_subscription(
            String,
            '/wachroboter/codeword',
            self.codeword_cb,
            10
        )

        self.create_subscription(
            Bool,
            '/wachroboter/alarm',
            self.alarm_cb,
            10
        )

        print("==========================================", flush=True)
        print(" WACHROBOTER ROS TOPIC MONITOR", flush=True)
        print("==========================================", flush=True)

    def timestamp(self):
        return f"{self.get_clock().now().nanoseconds / 1e9:.3f}"

    def guard_status_cb(self, msg):
        print(
            f"[{self.timestamp()}] "
            f"GUARD STATUS  -> {msg.data}",
            flush=True
        )

    def codeword_cb(self, msg):
        print(
            f"[{self.timestamp()}] "
            f"CODEWORD      -> {msg.data}",
            flush=True
        )

    def alarm_cb(self, msg):
        if msg.data:
            text = "TRUE  <<< ALARM >>>"
        else:
            text = "FALSE"

        print(
            f"[{self.timestamp()}] "
            f"ALARM         -> {text}",
            flush=True
        )


def main():
    rclpy.init()
    node = WachroboterMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
