#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class SequentialJointRotator(Node):
    def __init__(self):
        super().__init__('sequential_joint_rotator')
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

    def rotate_joints_sequentially(self):
        for joint in self.joint_names:
            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            point = JointTrajectoryPoint()

            positions = [0.0 for _ in self.joint_names]
            index = self.joint_names.index(joint)

            # Rotate current joint to 0.8 rad
            positions[index] = 0.8

            point.positions = positions
            point.time_from_start.sec = 2
            traj.points = [point]

            self.get_logger().info(f'Rotating joint: {joint} to position 0.8 rad')
            self.pub.publish(traj)

            # Wait for 3 seconds to observe the movement
            time.sleep(3)

            # Return joint to zero before next
            positions[index] = 0.0
            point.positions = positions
            traj.points = [point]
            self.pub.publish(traj)
            self.get_logger().info(f'Resetting joint: {joint} to position 0.0 rad')
            time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    node = SequentialJointRotator()

    try:
        node.rotate_joints_sequentially()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
