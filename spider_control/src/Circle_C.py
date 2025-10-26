#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class CircularGaitPublisher(Node):
    def __init__(self):
        super().__init__('circular_gait_publisher')
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

        # Left side legs (inner circle - shorter stride)
        self.left_legs = [0, 1, 2, 3]
        # Right side legs (outer circle - longer stride)
        self.right_legs = [4, 5, 6, 7]
        
        # Diagonal pairs for stability
        self.leg_pairs = [
            ([0, 2, 5, 7], [1, 3, 4, 6]),
            ([1, 3, 4, 6], [0, 2, 5, 7])
        ]
        
        self.state = 0
        self.substep = 0
        
        # Circular motion parameters
        self.inner_stride = 0.15   # Left side (inner) - SHORTER stride
        self.outer_stride = 0.35   # Right side (outer) - LONGER stride
        self.lift_height = 0.35
        
        # Turn direction: 1 = turn left (counter-clockwise), -1 = turn right (clockwise)
        self.turn_direction = -1  # Change to -1 for opposite direction
        
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info(f'Circular gait - Turn direction: {"LEFT" if self.turn_direction == 1 else "RIGHT"}')

    def get_coxa_angle(self, leg_idx, direction):
        """
        Get coxa angle with different strides for inner/outer legs
        """
        is_left = leg_idx in self.left_legs
        
        # Determine stride based on turn direction and leg side
        if self.turn_direction == 1:  # Turning left
            stride = self.inner_stride if is_left else self.outer_stride
        else:  # Turning right
            stride = self.outer_stride if is_left else self.inner_stride
        
        if direction == 'forward':
            return stride
        else:  # backward
            return -stride

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)

        active_legs, support_legs = self.leg_pairs[self.state]

        if self.substep == 0:
            # Lift active legs
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

            self.get_logger().info(f'State {self.state} - Lift active')

        elif self.substep == 1:
            # Swing active backward, support push forward
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

            self.get_logger().info(f'State {self.state} - Swing & push')

        else:  # substep == 2
            # Lower active legs
            for leg in active_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.get_coxa_angle(leg, 'backward')
                positions[femur_idx] = 0.0
            
            for leg in support_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.get_coxa_angle(leg, 'forward')
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
    node = CircularGaitPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()