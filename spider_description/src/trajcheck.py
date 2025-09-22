#!/usr/bin/env python3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rclpy
from rclpy.node import Node

class PreconfigureJoints(Node):
    def __init__(self):
        super().__init__('preconfigure_joints')
        self.pub = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.sent = False

    def timer_callback(self):
        if self.sent:
            return
        traj = JointTrajectory()
        traj.joint_names = [
            'joint1_coxa', 'joint2_coxa', 'joint3_coxa', 'joint4_coxa',
            'joint5_coxa', 'joint6_coxa', 'joint7_coxa', 'joint8_coxa'
        ]
        point = JointTrajectoryPoint()
        point.positions = [1.57, 1.57, 1.57, 1.57, -1.57, -1.57, -1.57, -1.57]
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.pub.publish(traj)
        self.get_logger().info("Preconfigured joint positions sent!")
        self.sent = True
