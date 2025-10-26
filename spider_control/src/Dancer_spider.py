#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class DanceGaitPublisher(Node):
    def __init__(self):
        super().__init__('dance_gait_publisher')
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

        # Left and right legs
        self.left_legs = [0, 1, 2, 3]
        self.right_legs = [4, 5, 6, 7]
        self.front_legs = [0, 4]
        self.back_legs = [3, 7]
        
        self.dance_move = 0
        self.timer = self.create_timer(0.4, self.timer_callback)
        self.get_logger().info('🎵 DANCE MODE ACTIVATED! 🎵')

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)

        if self.dance_move == 0:
            # MOVE 1: Wave - Left side up, right side down
            for leg in self.left_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = -0.3
                positions[femur_idx] = -0.5
            for leg in self.right_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = 0.3
                positions[femur_idx] = 0.5
            self.get_logger().info('💃 WAVE LEFT!')

        elif self.dance_move == 1:
            # MOVE 2: Wave - Right side up, left side down
            for leg in self.left_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = -0.3
                positions[femur_idx] = 0.5
            for leg in self.right_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = 0.3
                positions[femur_idx] = -0.5
            self.get_logger().info('💃 WAVE RIGHT!')

        elif self.dance_move == 2:
            # MOVE 3: Twist - All coxa rotate left
            for leg in range(8):
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = 0.4
                positions[femur_idx] = 0.0
            self.get_logger().info('🌀 TWIST LEFT!')

        elif self.dance_move == 3:
            # MOVE 4: Twist - All coxa rotate right
            for leg in range(8):
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = -0.4
                positions[femur_idx] = 0.0
            self.get_logger().info('🌀 TWIST RIGHT!')

        elif self.dance_move == 4:
            # MOVE 5: Stomp - Alternating legs up
            for i, leg in enumerate(range(8)):
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                tibia_idx = coxa_idx + 2
                if i % 2 == 0:
                    positions[femur_idx] = -0.6
                    positions[tibia_idx] = -0.3
                else:
                    positions[femur_idx] = 0.0
            self.get_logger().info('👟 STOMP 1!')

        elif self.dance_move == 5:
            # MOVE 6: Stomp - Opposite alternating
            for i, leg in enumerate(range(8)):
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                tibia_idx = coxa_idx + 2
                if i % 2 == 1:
                    positions[femur_idx] = -0.6
                    positions[tibia_idx] = -0.3
                else:
                    positions[femur_idx] = 0.0
            self.get_logger().info('👟 STOMP 2!')

        elif self.dance_move == 6:
            # MOVE 7: Front legs up - "Hands up!"
            for leg in self.front_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                tibia_idx = coxa_idx + 2
                positions[femur_idx] = -0.8
                positions[tibia_idx] = -0.5
            for leg in self.back_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[femur_idx] = 0.3
            self.get_logger().info('🙌 HANDS UP!')

        elif self.dance_move == 7:
            # MOVE 8: Back legs up - "Twerk!"
            for leg in self.back_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                tibia_idx = coxa_idx + 2
                positions[femur_idx] = -0.8
                positions[tibia_idx] = -0.5
            for leg in self.front_legs:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[femur_idx] = 0.3
            self.get_logger().info('🍑 BACK IT UP!')

        elif self.dance_move == 8:
            # MOVE 9: Diagonal stretch
            for leg in [0, 2, 5, 7]:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = 0.5
                positions[femur_idx] = -0.4
            for leg in [1, 3, 4, 6]:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = -0.5
                positions[femur_idx] = 0.4
            self.get_logger().info('✨ DIAGONAL STRETCH 1!')

        else:  # dance_move == 9
            # MOVE 10: Opposite diagonal stretch
            for leg in [1, 3, 4, 6]:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = 0.5
                positions[femur_idx] = -0.4
            for leg in [0, 2, 5, 7]:
                coxa_idx = leg * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = -0.5
                positions[femur_idx] = 0.4
            self.get_logger().info('✨ DIAGONAL STRETCH 2!')

        # Publish
        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 300_000_000
        traj.points = [point]
        self.pub.publish(traj)

        # Next move
        self.dance_move += 1
        if self.dance_move > 9:
            self.dance_move = 0


def main(args=None):
    rclpy.init(args=args)
    node = DanceGaitPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()