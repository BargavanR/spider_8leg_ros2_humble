#!/usr/bin/env python3
# '''
# *****************************************************************************************
# *  Filename:       circle_gait.py
# *  Description:    Improved circular gait with femur lift for realistic walking
# *  Created by:     BARGAVAN R
# *  Author:         SPIDER TEAM - MIT
# *****************************************************************************************
# '''
# import rclpy
# from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# class SequentialGaitPublisher(Node):
#     def __init__(self):
#         super().__init__('sequential_gait_publisher')
#         self.pub = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)

#         # Define coxa and femur joints
#         self.legs_coxa = [
#             'joint1_coxa', 'joint2_coxa', 'joint3_coxa', 'joint4_coxa',
#             'joint5_coxa', 'joint6_coxa', 'joint7_coxa', 'joint8_coxa'
#         ]
#         self.legs_femur = [
#             'joint1_fumer', 'joint2_fumer', 'joint3_fumer', 'joint4_fumer',
#             'joint5_fumer', 'joint6_fumer', 'joint7_fumer', 'joint8_fumer'
#         ]

#         # Full joint list
#         self.joint_names = [
#             'joint1_coxa', 'joint1_fumer', 'joint1_tibia',
#             'joint2_coxa', 'joint2_fumer', 'joint2_tibia',
#             'joint3_coxa', 'joint3_fumer', 'joint3_tibia',
#             'joint4_coxa', 'joint4_fumer', 'joint4_tibia',
#             'joint5_coxa', 'joint5_fumer', 'joint5_tibia',
#             'joint6_coxa', 'joint6_fumer', 'joint6_tibia',
#             'joint7_coxa', 'joint7_fumer', 'joint7_tibia',
#             'joint8_coxa', 'joint8_fumer', 'joint8_tibia'
#         ]

#         # Leg pairs: (front, back)
#         self.seq = [(0, 4), (1, 5), (2, 6), (3, 7)]
#         self.current_step = 0
#         self.forward_phase = False  # to alternate direction

#         # Faster gait timing (every 0.3s)
#         self.timer = self.create_timer(0.3, self.timer_callback)

#     def timer_callback(self):
#         traj = JointTrajectory()
#         traj.joint_names = self.joint_names
#         point = JointTrajectoryPoint()
#         positions = [0.0] * len(self.joint_names)

#         leg_pair = self.seq[self.current_step]
#         front_idx, back_idx = leg_pair

#         # --- FRONT LEG (swing phase) ---
#         coxa_f_idx = self.joint_names.index(self.legs_coxa[front_idx])
#         femur_f_idx = self.joint_names.index(self.legs_femur[front_idx])

#         if self.forward_phase:
#             positions[coxa_f_idx] = -1.57  # forward swing
#             positions[femur_f_idx] = 0.6  # lift leg up
#         else:
#             positions[coxa_f_idx] = 0.0   # reset to center
#             positions[femur_f_idx] = 0.0  # lower leg

#         # --- BACK LEG (support phase) ---
#         coxa_b_idx = self.joint_names.index(self.legs_coxa[back_idx])
#         femur_b_idx = self.joint_names.index(self.legs_femur[back_idx])

#         if self.forward_phase:
#             positions[coxa_b_idx] = 0.0   # stays centered
#             positions[femur_b_idx] = 0.0  # stays on ground
#         else:
#             positions[coxa_b_idx] = -1.57  # back swing
#             positions[femur_b_idx] = 0.0

#         point.positions = positions
#         point.time_from_start.sec = 0
#         point.time_from_start.nanosec = 300_000_000  # 0.3s
#         traj.points = [point]

#         self.pub.publish(traj)
#         self.get_logger().info(f'Gait step: pair ({front_idx+1}, {back_idx+1}) | forward_phase={self.forward_phase}')

#         # Alternate direction and switch pair
#         self.forward_phase = not self.forward_phase
#         if not self.forward_phase:
#             self.current_step = (self.current_step + 1) % len(self.seq)


# def main(args=None):
#     rclpy.init(args=args)
#     node = SequentialGaitPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
'''
*****************************************************************************************
*  Filename:       straight_gait.py
*  Description:    Corrected straight-line walking gait
*  Created by:     BARGAVAN R
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

        # Define which legs are on left (negative coxa) vs right (positive coxa)
        # Adjust these indices based on your robot's actual configuration
        # Assuming: 0,1,2,3 = left side, 4,5,6,7 = right side
        self.left_legs = [0, 1, 2, 3]   # Adjust to your robot
        self.right_legs = [4, 5, 6, 7]  # Adjust to your robot
        
        # Gait sequence: alternating tripod or wave pattern
        # For 8 legs, use pairs that maintain stability
        self.seq = [(0, 4), (1, 5), (2, 6), (3, 7)]
        
        self.current_step = 0
        self.forward_phase = False
        
        # Smaller coxa angle for forward walking (not turning)
        self.coxa_angle = 0.3  # ~17 degrees (much smaller than -1.57!)
        self.femur_lift = 0.4  # Lift height during swing
        
        self.timer = self.create_timer(0.3, self.timer_callback)

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)

        leg_pair = self.seq[self.current_step]
        front_idx, back_idx = leg_pair

        # Determine if legs are on left or right side
        front_is_left = front_idx in self.left_legs
        back_is_left = back_idx in self.left_legs

        # --- FRONT LEG (swing phase) ---
        coxa_f_idx = front_idx * 3  # Each leg has 3 joints
        femur_f_idx = coxa_f_idx + 1

        if self.forward_phase:
            # Swing forward: opposite directions for left/right
            positions[coxa_f_idx] = -self.coxa_angle if front_is_left else self.coxa_angle
            positions[femur_f_idx] = self.femur_lift  # Lift leg
        else:
            # Return to stance
            positions[coxa_f_idx] = self.coxa_angle if front_is_left else -self.coxa_angle
            positions[femur_f_idx] = 0.0  # Lower leg

        # --- BACK LEG (stance/support phase) ---
        coxa_b_idx = back_idx * 3
        femur_b_idx = coxa_b_idx + 1

        if self.forward_phase:
            # Push backward: opposite to swing direction
            positions[coxa_b_idx] = self.coxa_angle if back_is_left else -self.coxa_angle
            positions[femur_b_idx] = 0.0  # On ground
        else:
            # Prepare for next cycle
            positions[coxa_b_idx] = -self.coxa_angle if back_is_left else self.coxa_angle
            positions[femur_b_idx] = 0.0

        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 300_000_000
        traj.points = [point]

        self.pub.publish(traj)
        self.get_logger().info(
            f'Step: pair ({front_idx+1},{back_idx+1}) | '
            f'phase={self.forward_phase} | '
            f'coxa_angle={self.coxa_angle}'
        )

        self.forward_phase = not self.forward_phase
        if not self.forward_phase:
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