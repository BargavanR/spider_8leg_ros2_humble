#!/usr/bin/env python3
'''
*****************************************************************************************
*  Filename:       circle_gait.py
*  Description:    circle gait plan nadanchuuuu swagaaaa
*  created by:    BARGAVAN R
*  Author:         SPIDER TEAM - MIT
*****************************************************************************************
'''
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SequentialGaitPublisher(Node):
    def __init__(self):
        super().__init__('sequential_gait_publisher')
        self.pub = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)

        # Define coxa joints
        self.legs_coxa = [
            'joint1_coxa', 'joint2_coxa', 'joint3_coxa', 'joint4_coxa',
            'joint5_coxa', 'joint6_coxa', 'joint7_coxa', 'joint8_coxa'
        ]

        # Full joint list
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

        # Leg pairs: (front, back)
        self.seq = [(0, 4), (1, 5), (2, 6), (3, 7)]
        self.current_step = 0

        # Faster gait (0.3 sec instead of 1 sec)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()

        # Reset all to 0
        positions = [0.0] * len(self.joint_names)

        # Current leg pair
        leg_pair = self.seq[self.current_step]

        # Forward legs (1,2,3,4) swing forward (+45°)
        idx_f = self.joint_names.index(self.legs_coxa[leg_pair[0]])
        positions[idx_f] = 1.57#1.57 #0.78  # +45°

        # Back legs (5,6,7,8) swing backward (-45°)
        idx_b = self.joint_names.index(self.legs_coxa[leg_pair[1]])
        positions[idx_b] = -1.57 # -0.78  # -45°

        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 300_000_000  # 0.3s
        traj.points = [point]

        self.pub.publish(traj)
        self.get_logger().info(f'Moving pair {leg_pair[0]+1} & {leg_pair[1]+1}')

        # Next pair
        self.current_step = (self.current_step + 1) % len(self.seq)


def main(args=None):
    rclpy.init(args=args)
    node = SequentialGaitPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
