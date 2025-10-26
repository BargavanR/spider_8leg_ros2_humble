#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JumpGaitPublisher(Node):
    def __init__(self):
        super().__init__('jolly_gait_publisher')
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

        self.all_legs = [0, 1, 2, 3, 4, 5, 6, 7]
        self.jump_phase = 0
        
        self.timer = self.create_timer(0.25, self.timer_callback)
        self.get_logger().info('Jump with proper reset cycle!')

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)

        if self.jump_phase == 0:
            # PHASE 0: Stand neutral - RESET everything to zero
            for leg in self.all_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                tibia_idx = coxa_idx + 2
                positions[coxa_idx] = 0.0
                positions[femur_idx] = 0.0
                positions[tibia_idx] = 0.0
            
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 200_000_000
            self.get_logger().info('RESET: Standing neutral')

        elif self.jump_phase == 1:
            # PHASE 1: Crouch down
            for leg in self.all_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                tibia_idx = coxa_idx + 2
                positions[coxa_idx] = 0.0
                positions[femur_idx] = -0.6
                positions[tibia_idx] = -0.6
            
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 200_000_000
            self.get_logger().info('CROUCH')

        elif self.jump_phase == 2:
            # PHASE 2: JUMP - explosive extension
            for leg in self.all_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                tibia_idx = coxa_idx + 2
                positions[coxa_idx] = 0.0
                positions[femur_idx] = 0.8
                positions[tibia_idx] = 0.8
            
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 100_000_000  # Fast!
            self.get_logger().info('JUMP!')

        elif self.jump_phase == 3:
            # PHASE 3: Mid-air - tuck legs
            for leg in self.all_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                tibia_idx = coxa_idx + 2
                positions[coxa_idx] = 0.0
                positions[femur_idx] = 0.3
                positions[tibia_idx] = 0.3
            
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 150_000_000
            self.get_logger().info('AIRBORNE')

        elif self.jump_phase == 4:
            # PHASE 4: Extend for landing
            for leg in self.all_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                tibia_idx = coxa_idx + 2
                positions[coxa_idx] = 0.0
                positions[femur_idx] = -0.2
                positions[tibia_idx] = -0.2
            
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 150_000_000
            self.get_logger().info('LANDING')

        else:  # jump_phase == 5
            # PHASE 5: Compress on impact
            for leg in self.all_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                tibia_idx = coxa_idx + 2
                positions[coxa_idx] = 0.0
                positions[femur_idx] = -0.4
                positions[tibia_idx] = -0.4
            
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 200_000_000
            self.get_logger().info('ABSORB IMPACT')

        # Publish
        point.positions = positions
        traj.points = [point]
        self.pub.publish(traj)

        # Advance
        self.jump_phase += 1
        if self.jump_phase > 5:
            self.jump_phase = 0  # Back to neutral reset


def main(args=None):
    rclpy.init(args=args)
    node = JumpGaitPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()