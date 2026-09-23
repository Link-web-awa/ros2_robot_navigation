import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = None

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.sub = self.create_subscription(
            Twist, '/car/actual_velocity', self.on_velocity, 10)
        self.get_logger().info('Waiting for chassis velocity feedback')

    def on_velocity(self, velocity):
        vx = velocity.linear.x
        vy = velocity.linear.y
        wz = velocity.angular.z
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            return

        current = time.monotonic()
        dt = 0.0 if self.last_time is None else current - self.last_time
        self.last_time = current

        # Do not integrate across a feedback interruption.
        if dt > 0.5:
            self.get_logger().warning(
                'Velocity feedback interrupted; skipping integration')
            dt = 0.0

        # Rotate body-frame velocity into the odometry frame.
        middle_yaw = self.yaw + 0.5 * wz * dt
        self.x += (
            vx * math.cos(middle_yaw) - vy * math.sin(middle_yaw)
        ) * dt
        self.y += (
            vx * math.sin(middle_yaw) + vy * math.cos(middle_yaw)
        ) * dt

        angle = self.yaw + wz * dt
        self.yaw = math.atan2(math.sin(angle), math.cos(angle))

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.twist.twist = velocity

        # Uncalibrated placeholders, not measured sensor uncertainty.
        for index, variance in enumerate(
                [0.1, 0.1, 1e6, 1e6, 1e6, 0.1]):
            msg.pose.covariance[index * 7] = variance
        for index, variance in enumerate(
                [0.01, 0.01, 1e6, 1e6, 1e6, 0.02]):
            msg.twist.covariance[index * 7] = variance

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = WheelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
