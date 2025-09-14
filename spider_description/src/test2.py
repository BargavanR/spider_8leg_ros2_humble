#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class SpiderGaitPublisher(Node):
    def __init__(self):
        super().__init__('spider_gait_publisher')
        self.pub = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)

        # Define joint names in order: each leg has coxa, femur, tibia
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

        # Walking parameters
        self.freq = 1.0  # gait cycle frequency (Hz)
        self.amplitude_coxa = 0.7  # radians
        self.amplitude_femur = 0.8  # radians (lift)
        self.amplitude_tibia = 0.6  # radians (extension)

        # Leg groups with phase offsets
        self.group1 = [0, 1, 2, 6, 7, 8, 12, 13, 14, 18, 19, 20]  # legs 1,3,5,7 (their joints indexes)
        self.group2 = [3, 4, 5, 9, 10, 11, 15, 16, 17, 21, 22, 23]  # legs 2,4,6,8

        self.time_start = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

    def gait_angle(self, joint_index, t):
        # Determine which group joint belongs to
        if joint_index in self.group1:
            phase = 0.0
        elif joint_index in self.group2:
            phase = math.pi  # 180 degree phase shift
        else:
            phase = 0.0  # fallback

        segment = joint_index % 3  # 0=coxa, 1=femur, 2=tibia

        if segment == 0:
            # Coxa swings horizontally left-right
            return self.amplitude_coxa * math.sin(2 * math.pi * self.freq * t + phase)
        elif segment == 1:
            # Femur lifts leg up-down (half-wave rectified cosine)
            return self.amplitude_femur * (1 - math.cos(2 * math.pi * self.freq * t + phase)) / 2
        else:
            # Tibia extends/retracts synchronously with femur
            return self.amplitude_tibia * (1 - math.cos(2 * math.pi * self.freq * t + phase)) / 2

    def timer_callback(self):
        now = self.get_clock().now()
        t = (now - self.time_start).nanoseconds / 1e9  # seconds elapsed

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        positions = [self.gait_angle(i, t) for i in range(len(self.joint_names))]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1  # duration 1 sec for interpolation

        traj.points = [point]

        self.pub.publish(traj)
        self.get_logger().info(f'Published gait positions at t={t:.2f}s')

def main(args=None):
    rclpy.init(args=args)
    node = SpiderGaitPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
