#!/usr/bin/env python3

#Super _smotth straight
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MirrorGaitPublisher(Node):
    def __init__(self):
        super().__init__('mirror_gait_publisher')
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

        # Left side legs (rotate negative for forward)
        self.left_legs = [0, 1, 2, 3]  # Adjust based on your robot
        # Right side legs (rotate positive for forward)
        self.right_legs = [4, 5, 6, 7]  # Adjust based on your robot
        
        # Diagonal pairs for stability - mix left and right
        self.leg_pairs = [
            ([0, 2, 5, 7], [1, 3, 4, 6]),  # State A: Mixed diagonals
            ([1, 3, 4, 6], [0, 2, 5, 7])   # State B: Opposite diagonals
        ]
        
        self.state = 0
        self.substep = 0  # 0=lift, 1=swing, 2=lower
        
        # Parameters
        self.stride = 0.25
        self.lift_height = 0.35
        
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('Mirror gait with OPPOSITE coxa angles')

    def get_coxa_angle(self, leg_idx, direction):
        """
        Get coxa angle based on leg side and direction
        direction: 'forward' or 'backward'
        """
        is_left = leg_idx in self.left_legs
        
        if direction == 'forward':
            # Left legs: positive for forward
            # Right legs: positive for forward (SAME DIRECTION - your robot config)
            return self.stride
        else:  # backward
            return -self.stride

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)

        active_legs, support_legs = self.leg_pairs[self.state]

        if self.substep == 0:
            # SUBSTEP 0: Lift active legs, support legs at back position
            for leg in active_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.get_coxa_angle(leg, 'backward')
                positions[femur_idx] = self.lift_height
            
            for leg in support_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.get_coxa_angle(leg, 'forward')
                positions[femur_idx] = 0.0

            self.get_logger().info(f'State {self.state} - Lift active')

        elif self.substep == 1:
            # SUBSTEP 1: Swing active forward (lifted), support push back
            for leg in active_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.get_coxa_angle(leg, 'forward')
                positions[femur_idx] = self.lift_height
            
            for leg in support_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.get_coxa_angle(leg, 'backward')
                positions[femur_idx] = 0.0

            self.get_logger().info(f'State {self.state} - Swing & push')

        else:  # substep == 2
            # SUBSTEP 2: Lower active legs
            for leg in active_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.get_coxa_angle(leg, 'forward')
                positions[femur_idx] = 0.0
            
            for leg in support_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.get_coxa_angle(leg, 'backward')
                positions[femur_idx] = 0.0

            self.get_logger().info(f'State {self.state} - Lower active')

        # Publish
        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200_000_000
        traj.points = [point]
        self.pub.publish(traj)

        # Advance
        self.substep += 1
        if self.substep > 2:
            self.substep = 0
            self.state = 1 - self.state


def main(args=None):
    rclpy.init(args=args)
    node = MirrorGaitPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()


