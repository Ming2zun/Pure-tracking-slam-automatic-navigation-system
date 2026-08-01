# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import TransformStamped
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener
# from tf_transformations import euler_from_quaternion
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float32
# import math


# class AckermannSteeringAngle(Node):
#     def __init__(self):
#         super().__init__('ackermann_steering_angle')

#         self.get_logger().info('阿克曼转向角度监测节点启动')

#         self.declare_parameter('reference_frame', 'base_link')
#         self.declare_parameter('left_steering_link', 'front_left_steering_link')
#         self.declare_parameter('right_steering_link', 'front_right_steering_link')
#         self.declare_parameter('publish_rate', 20.0)

#         self.reference_frame = self.get_parameter('reference_frame').value
#         self.left_link = self.get_parameter('left_steering_link').value
#         self.right_link = self.get_parameter('right_steering_link').value
#         self.publish_rate = self.get_parameter('publish_rate').value

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self.left_angle_pub = self.create_publisher(
#             Float32, '/vehicle/front_left_steering_angle', 10)
#         self.right_angle_pub = self.create_publisher(
#             Float32, '/vehicle/front_right_steering_angle', 10)
#         self.avg_angle_pub = self.create_publisher(
#             Float32, '/vehicle/steering_angle', 10)

#         self.joint_state_sub = self.create_subscription(
#             JointState, '/joint_states', self.joint_state_callback, 10)
#         self.joint_left_angle = None
#         self.joint_right_angle = None

#         self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

#         self.get_logger().info(
#             f'参考坐标系: {self.reference_frame}\n'
#             f'左转向link: {self.left_link}\n'
#             f'右转向link: {self.right_link}\n'
#             f'发布频率: {self.publish_rate} Hz')

#     def lookup_steering_angle(self, target_frame):
#         try:
#             transform = self.tf_buffer.lookup_transform(
#                 self.reference_frame,
#                 target_frame,
#                 rclpy.time.Time(),
#                 rclpy.duration.Duration(seconds=0.1))
#         except Exception as ex:
#             self.get_logger().debug(
#                 f'无法获取 {self.reference_frame} -> {target_frame} 的变换: {ex}')
#             return None

#         q = transform.transform.rotation
#         quat = [q.x, q.y, q.z, q.w]
#         roll, pitch, yaw = euler_from_quaternion(quat)

#         return yaw

#     def timer_callback(self):
#         left_angle = self.lookup_steering_angle(self.left_link)
#         right_angle = self.lookup_steering_angle(self.right_link)

#         if left_angle is not None and right_angle is not None:
#             left_msg = Float32()
#             left_msg.data = left_angle
#             self.left_angle_pub.publish(left_msg)

#             right_msg = Float32()
#             right_msg.data = right_angle
#             self.right_angle_pub.publish(right_msg)

#             avg_angle = (left_angle + right_angle) / 2.0
#             avg_msg = Float32()
#             avg_msg.data = avg_angle
#             self.avg_angle_pub.publish(avg_msg)

#             self.get_logger().info(
#                 f'[TF] 左: {math.degrees(left_angle):.2f} deg  '
#                 f'右: {math.degrees(right_angle):.2f} deg  '
#                 f'平均: {math.degrees(avg_angle):.2f} deg')

#             if self.joint_left_angle is not None and self.joint_right_angle is not None:
#                 self.get_logger().info(
#                     f'[Joint] 左: {math.degrees(self.joint_left_angle):.2f} deg  '
#                     f'右: {math.degrees(self.joint_right_angle):.2f} deg')

#     def joint_state_callback(self, msg):
#         try:
#             fl_idx = msg.name.index('front_left_steering_joint')
#             fr_idx = msg.name.index('front_right_steering_joint')
#             self.joint_left_angle = msg.position[fl_idx]
#             self.joint_right_angle = msg.position[fr_idx]
#         except (ValueError, IndexError):
#             pass


# def main(args=None):
#     rclpy.init(args=args)
#     node = AckermannSteeringAngle()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import math


class AckermannSteeringAngle(Node):
    def __init__(self):
        super().__init__('ackermann_steering_angle')

        self.get_logger().info('阿克曼转向角度监测节点启动')

        self.declare_parameter('reference_frame', 'base_link')
        self.declare_parameter('left_steering_link', 'front_left_steering_link')
        self.declare_parameter('right_steering_link', 'front_right_steering_link')
        # 修改默认频率为 2Hz
        self.declare_parameter('publishing_rate', 2.0)

        self.reference_frame = self.get_parameter('reference_frame').value
        self.left_link = self.get_parameter('left_steering_link').value
        self.right_link = self.get_parameter('right_steering_link').value
        self.publish_rate = self.get_parameter('publishing_rate').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.left_angle_pub = self.create_publisher(
            Float32, '/vehicle/front_left_steering_angle', 10)
        self.right_angle_pub = self.create_publisher(
            Float32, '/vehicle/front_right_steering_angle', 10)
        self.avg_angle_pub = self.create_publisher(
            Float32, '/vehicle/steering_angle', 10)

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.joint_left_angle = None
        self.joint_right_angle = None

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'参考坐标系: {self.reference_frame}\n'
            f'左转向link: {self.left_link}\n'
            f'右转向link: {self.right_link}\n'
            f'发布频率: {self.publish_rate} Hz')

    def lookup_steering_angle(self, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                target_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.2))
        except Exception as ex:
            self.get_logger().debug(
                f'无法获取 {self.reference_frame} -> {target_frame} 的变换: {ex}')
            return None

        q = transform.transform.rotation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat)

        # 直接返回角度
        return math.degrees(yaw)

    def timer_callback(self):
        left_angle_deg = self.lookup_steering_angle(self.left_link)
        right_angle_deg = self.lookup_steering_angle(self.right_link)

        if left_angle_deg is not None and right_angle_deg is not None:
            left_msg = Float32()
            left_msg.data = left_angle_deg
            self.left_angle_pub.publish(left_msg)

            right_msg = Float32()
            right_msg.data = right_angle_deg
            self.right_angle_pub.publish(right_msg)

            avg_angle_deg = (left_angle_deg + right_angle_deg) / 2.0
            avg_msg = Float32()
            avg_msg.data = avg_angle_deg
            self.avg_angle_pub.publish(avg_msg)

            # 日志直接打印角度
            self.get_logger().info(
                f'[TF方式] 左转向: {left_angle_deg:.2f}°  '
                f'右转向: {right_angle_deg:.2f}°')

            if self.joint_left_angle is not None and self.joint_right_angle is not None:
                joint_left_deg = math.degrees(self.joint_left_angle)
                joint_right_deg = math.degrees(self.joint_right_angle)
                self.get_logger().info(
                    f'[Joint方式] 左转向: {joint_left_deg:.2f}°  '
                    f'右转向: {joint_right_deg:.2f}°')

    def joint_state_callback(self, msg):
        try:
            fl_idx = msg.name.index('front_left_steering_joint')
            fr_idx = msg.name.index('front_right_steering_joint')
            self.joint_left_angle = msg.position[fl_idx]
            self.joint_right_angle = msg.position[fr_idx]
        except (ValueError, IndexError):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = AckermannSteeringAngle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
