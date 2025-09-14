#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class AggressiveGaitPublisher(Node):
    def __init__(self):
        super().__init__('aggressive_gait_publisher')
        self.pub = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)

        self.joint_names = [
            'joint1_coxa', 'joint1_fumer', 'joint1_tibia',
            'joint2_coxa', 'joint2_fumer', 'joint2_tibia',
            'joint3_coxa', 'joint3_fumer', 'joint3_tibia',
            'joint4_coxa', 'joint4_fumer', 'joint4_tibia',
            'joint5_coxa', 'joint5_fumer', 'joint5_tibia',
            'joint6_coxa', 'joint6_fumer', 'joint6_tibia',
            'joint7_coxa', 'joint7_fumer', 'joint7_tibia',
            'joint8_coxa', 'joint8_fumer', 'joint8_tibia'
        ]

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz publish rate
        self.time_start = self.get_clock().now()

    def timer_callback(self):
        t = (self.get_clock().now() - self.time_start).nanoseconds / 1e9  # time in seconds
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()

        positions = []
        for i, _ in enumerate(self.joint_names):
            phase = (i % 3) * (2 * math.pi / 3)
            # Increase amplitude and frequency for aggressive gait
            pos = 1.0 * math.sin(2 * math.pi * 1.5 * t + phase)  # 1.0 rad amplitude, 1.5 Hz frequency
            positions.append(pos)

        point.positions = positions
        point.time_from_start.sec = 1
        traj.points = [point]

        self.pub.publish(traj)
        self.get_logger().info(f'Publishing aggressive gait at time {t:.2f}s')

def main(args=None):
    rclpy.init(args=args)
    node = AggressiveGaitPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
