#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import LaserScan

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw):
    z = math.sin(yaw / 2.0)
    w = math.cos(yaw / 2.0)
    return z, w


class FakeRobot(Node):

    def __init__(self):
        super().__init__('fake_robot')

        self.declare_parameter('robot_name', 'robot')
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('yaw0', 0.0)
        self.declare_parameter('linear_speed', 0.40)
        self.declare_parameter('angular_speed', 1.2)

        self.robot_name = (
            self.get_parameter('robot_name')
            .get_parameter_value()
            .string_value
        )

        self.x = (
            self.get_parameter('x0')
            .get_parameter_value()
            .double_value
        )

        self.y = (
            self.get_parameter('y0')
            .get_parameter_value()
            .double_value
        )

        self.yaw = (
            self.get_parameter('yaw0')
            .get_parameter_value()
            .double_value
        )

        self.linear_speed = (
            self.get_parameter('linear_speed')
            .get_parameter_value()
            .double_value
        )

        self.angular_speed = (
            self.get_parameter('angular_speed')
            .get_parameter_value()
            .double_value
        )

        self.v = 0.0
        self.w = 0.0

        self.lock = threading.Lock()

        # Unique TF frame names for each simulated robot
        self.odom_frame = f'{self.robot_name}_odom_combined'
        self.base_frame = f'{self.robot_name}_base_footprint'
        self.base_link = f'{self.robot_name}_base_link'
        self.laser_frame = f'{self.robot_name}_laser'

        self.odom_pub = self.create_publisher(
            Odometry, 'odom', 10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped, 'pose', 10
        )

        self.scan_pub = self.create_publisher(
            LaserScan, 'scan', 10
        )

        self.velocity_pub = self.create_publisher(
            Twist, 'cmd_vel_sim', 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)

        self.publish_static_transforms()

        self.action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_goal,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.timer = self.create_timer(
            0.05,
            self.publish_state
        )

        self.get_logger().info(
            f'Fake {self.robot_name} started at '
            f'x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}'
        )

    def publish_static_transforms(self):

        transforms = []

        # map -> robot odometry
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = self.odom_frame
        t.transform.rotation.w = 1.0
        transforms.append(t)

        # base_footprint -> base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.base_link
        t.transform.rotation.w = 1.0
        transforms.append(t)

        # base_link -> laser
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_link
        t.child_frame_id = self.laser_frame

        t.transform.translation.x = 0.225

        z, w = quaternion_from_yaw(math.pi)
        t.transform.rotation.z = z
        t.transform.rotation.w = w

        transforms.append(t)

        self.static_tf.sendTransform(transforms)

    def make_pose(self):
        pose = PoseStamped()

        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        with self.lock:
            pose.pose.position.x = self.x
            pose.pose.position.y = self.y

            z, w = quaternion_from_yaw(self.yaw)

            pose.pose.orientation.z = z
            pose.pose.orientation.w = w

        return pose

    def publish_state(self):

        now = self.get_clock().now().to_msg()

        with self.lock:
            x = self.x
            y = self.y
            yaw = self.yaw
            v = self.v
            w_vel = self.w

        zq, wq = quaternion_from_yaw(yaw)

        # TF odom -> base
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame

        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = zq
        tf.transform.rotation.w = wq

        self.tf_broadcaster.sendTransform(tf)

        # Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = zq
        odom.pose.pose.orientation.w = wq

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w_vel

        self.odom_pub.publish(odom)

        # Pose directly in map
        self.pose_pub.publish(self.make_pose())

        # Fake clear laser
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = self.laser_frame

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / 719.0

        scan.range_min = 0.05
        scan.range_max = 25.0

        scan.ranges = [float('inf')] * 720

        self.scan_pub.publish(scan)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w_vel
        self.velocity_pub.publish(cmd)

    def goal_callback(self, goal_request):

        p = goal_request.pose.pose.position

        self.get_logger().info(
            f'Goal received: x={p.x:.3f}, y={p.y:.3f}'
        )

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_goal(self, goal_handle):

        target = goal_handle.request.pose.pose

        tx = target.position.x
        ty = target.position.y

        target_yaw = yaw_from_quaternion(
            target.orientation
        )

        dt = 0.05

        # Move toward target
        while rclpy.ok():

            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.v = 0.0
                    self.w = 0.0

                goal_handle.canceled()
                return NavigateToPose.Result()

            with self.lock:
                dx = tx - self.x
                dy = ty - self.y

                distance = math.hypot(dx, dy)

                if distance < 0.04:
                    self.v = 0.0
                    self.w = 0.0
                    break

                desired_yaw = math.atan2(dy, dx)

                error = normalize_angle(
                    desired_yaw - self.yaw
                )

                # Rotate first if necessary
                if abs(error) > 0.18:

                    step = max(
                        -self.angular_speed * dt,
                        min(
                            self.angular_speed * dt,
                            error
                        )
                    )

                    self.yaw = normalize_angle(
                        self.yaw + step
                    )

                    self.v = 0.0
                    self.w = step / dt

                else:

                    speed = min(
                        self.linear_speed,
                        distance
                    )

                    self.x += speed * math.cos(
                        self.yaw
                    ) * dt

                    self.y += speed * math.sin(
                        self.yaw
                    ) * dt

                    self.v = speed
                    self.w = 0.0

            feedback = NavigateToPose.Feedback()
            feedback.current_pose = self.make_pose()
            feedback.distance_remaining = float(distance)

            goal_handle.publish_feedback(feedback)

            time.sleep(dt)

        # Rotate to requested final orientation
        while rclpy.ok():

            with self.lock:
                error = normalize_angle(
                    target_yaw - self.yaw
                )

                if abs(error) < 0.03:
                    self.yaw = target_yaw
                    self.v = 0.0
                    self.w = 0.0
                    break

                step = max(
                    -self.angular_speed * dt,
                    min(
                        self.angular_speed * dt,
                        error
                    )
                )

                self.yaw = normalize_angle(
                    self.yaw + step
                )

                self.v = 0.0
                self.w = step / dt

            time.sleep(dt)

        self.get_logger().info(
            f'Goal reached: x={tx:.3f}, y={ty:.3f}'
        )

        goal_handle.succeed()

        return NavigateToPose.Result()


def main():

    rclpy.init()

    node = FakeRobot()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
