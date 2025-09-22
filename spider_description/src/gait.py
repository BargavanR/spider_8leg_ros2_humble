#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# import math

# class SequentialGaitPublisher(Node):
#     def __init__(self):
#         super().__init__('sequential_gait_publisher')
#         self.pub = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)

#         # Define leg joints in sequence
#         self.legs = [
#             ['joint1_coxa', 'joint1_fumer', 'joint1_tibia'],
#             ['joint2_coxa', 'joint2_fumer', 'joint2_tibia'],
#             ['joint3_coxa', 'joint3_fumer', 'joint3_tibia'],
#             ['joint4_coxa', 'joint4_fumer', 'joint4_tibia'],
#             ['joint5_coxa', 'joint5_fumer', 'joint5_tibia'],
#             ['joint6_coxa', 'joint6_fumer', 'joint6_tibia'],
#             ['joint7_coxa', 'joint7_fumer', 'joint7_tibia'],
#             ['joint8_coxa', 'joint8_fumer', 'joint8_tibia']
#         ]

#         # Flatten joint names
#         self.joint_names = [joint for leg in self.legs for joint in leg]

#         self.current_leg = 0  # start with first leg
#         self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz step for sequential movement

#     def timer_callback(self):
#         traj = JointTrajectory()
#         traj.joint_names = self.joint_names
#         point = JointTrajectoryPoint()

#         positions = [0.0] * len(self.joint_names)  # reset all joints

#         # Move current leg
# #         leg_joints = self.legs[self.current_leg]
# #         for joint in leg_joints:
# #             idx = self.joint_names.index(joint)
# #             positions[idx] = 1.57  # move 90 degrees

# #         point.positions = positions
# #         point.time_from_start.sec = 1
# #         traj.points = [point]

# #         self.pub.publish(traj)
# #         self.get_logger().info(f'Moving leg {self.current_leg + 1}/{len(self.legs)}')

# #         # Move to next leg in next callback
# #         self.current_leg = (self.current_leg + 1) % len(self.legs)

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = SequentialGaitPublisher()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()
# import rclpy
# from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# class SequentialGaitPublisher(Node):
#     def __init__(self):
#         super().__init__('sequential_gait_publisher')
#         self.pub = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)

#         # Define leg joints (only coxa for movement)
#         self.legs_coxa = [
#             'joint1_coxa', 'joint2_coxa', 'joint3_coxa', 'joint4_coxa',
#             'joint5_coxa', 'joint6_coxa', 'joint7_coxa', 'joint8_coxa'
#         ]

#         # Full joint list (coxa + femur + tibia) for trajectory
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

#         # Desired sequence: 1,5,2,6,3,7,4,8 (0-indexed in Python)
#         self.sequence = [0, 4, 1, 5, 2, 6, 3, 7]
#         self.seq=[(1,5),(2,6),(3,7),(4,8)]
#         self.current_step = 0
#         self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz step

#     def timer_callback(self):
#         traj = JointTrajectory()
#         traj.joint_names = self.joint_names
#         point = JointTrajectoryPoint()
#         '''
#         here the self.seq is the concept where i want my coxa's to move in in directions and joint 1 coxa moves to +45 deg and joint 5 moves simultNEOUSLY AND WHEN joint 2 and 6 moves and that time  those 1 and 5 comes 0 and then this seq continous 
#         '''
#         # Reset all joints to 0
#         positions = [0.0] * len(self.joint_names)

#         # Move only the coxa joint in the current sequence
#         leg_idx = self.sequence[self.current_step]
#         coxa_joint = self.legs_coxa[leg_idx]
#         idx = self.joint_names.index(coxa_joint)
#         positions[idx] = 1.57  # move 90 degrees

#         point.positions = positions
#         point.time_from_start.sec = 1
#         traj.points = [point]

#         self.pub.publish(traj)
#         self.get_logger().info(f'Moving leg {leg_idx + 1}/8 (coxa only)')

#         # Move to next leg in the sequence
#         self.current_step = (self.current_step + 1) % len(self.sequence)

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
# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# class SequentialGaitPublisher(Node):
#     def __init__(self):
#         super().__init__('sequential_gait_publisher')
#         self.pub = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)

#         # Coxa joints
#         self.coxa_joints = [
#             'joint1_coxa', 'joint2_coxa', 'joint3_coxa', 'joint4_coxa',
#             'joint5_coxa', 'joint6_coxa', 'joint7_coxa', 'joint8_coxa'
#         ]

#         # Full joint list (coxa + femur + tibia)
#         self.joint_names = [
#             'joint1_coxa','joint1_fumer','joint1_tibia',
#             'joint2_coxa','joint2_fumer','joint2_tibia',
#             'joint3_coxa','joint3_fumer','joint3_tibia',
#             'joint4_coxa','joint4_fumer','joint4_tibia',
#             'joint5_coxa','joint5_fumer','joint5_tibia',
#             'joint6_coxa','joint6_fumer','joint6_tibia',
#             'joint7_coxa','joint7_fumer','joint7_tibia',
#             'joint8_coxa','joint8_fumer','joint8_tibia'
#         ]

#         # Sequence alternating between group 1(+90°) and group 2(-90°)
#         self.sequence = [0, 4, 1, 5, 2, 6, 3, 7]  # 0-indexed
#         self.current_step = 0

#         # Store positions (all start at 0)
#         self.positions = [0.0] * len(self.joint_names)

#         # Timer for sequential movement
#         self.timer = self.create_timer(1.0, self.timer_callback)

#     def timer_callback(self):
#         traj = JointTrajectory()
#         traj.joint_names = self.joint_names
#         point = JointTrajectoryPoint()

#         # Current leg in sequence
#         leg_idx = self.sequence[self.current_step]
#         coxa_joint = self.coxa_joints[leg_idx]
#         idx = self.joint_names.index(coxa_joint)

#         # Apply +90° for legs 1-4, -90° for legs 5-8
#         if leg_idx < 4:
#             self.positions[idx] = 0.7  # +90°
#         else:
#             self.positions[idx] = -0.7  # -90° inverted

#         # Femur and tibia remain 0
#         point.positions = self.positions
#         point.time_from_start.sec = 1
#         traj.points = [point]

#         self.pub.publish(traj)
#         self.get_logger().info(
#             f'Moved leg {leg_idx + 1} coxa to {"+90°" if leg_idx < 4 else "-90°"}'
#         )

#         # Next leg in sequence
#         self.current_step = (self.current_step + 1) % len(self.sequence)


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

# import rclpy
# from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# class SequentialGaitPublisher(Node):
#     def __init__(self):
#         super().__init__('sequential_gait_publisher')
#         self.pub = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)

#         # Define only coxa joints
#         self.legs_coxa = [
#             'joint1_coxa', 'joint2_coxa', 'joint3_coxa', 'joint4_coxa',
#             'joint5_coxa', 'joint6_coxa', 'joint7_coxa', 'joint8_coxa'
#         ]

#         # Full joint list (coxa + femur + tibia) for trajectory
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

#         # Pairs of legs to move together
#         # (front leg idx, back leg idx) -- Python is 0-indexed
#         self.seq = [(0, 4), (1, 5), (2, 6), (3, 7)]

#         self.current_step = 0
#         self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz

#     def timer_callback(self):
#         traj = JointTrajectory()
#         traj.joint_names = self.joint_names
#         point = JointTrajectoryPoint()

#         # Reset all joints to 0 at every step
#         positions = [0.0] * len(self.joint_names)

#         # Get the pair of legs for this step
#         front_leg, back_leg = self.seq[self.current_step]

#         # Move front leg (1–4) to +45° (0.785 rad)
#         front_idx = self.joint_names.index(self.legs_coxa[front_leg])
#         positions[front_idx] = 0.785  # +45°

#         # Move back leg (5–8) to -45° (-0.785 rad)
#         back_idx = self.joint_names.index(self.legs_coxa[back_leg])
#         positions[back_idx] = -0.785  # -45°

#         # Send trajectory
#         point.positions = positions
#         point.time_from_start.sec = 1
#         traj.points = [point]
#         self.pub.publish(traj)

#         self.get_logger().info(
#             f"Step {self.current_step+1}/{len(self.seq)}: "
#             f"Leg {front_leg+1} → +45°, Leg {back_leg+1} → -45°"
#         )

#         # Next step
#         self.current_step = (self.current_step + 1) % len(self.seq)

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
        self.timer = self.create_timer(0.3, self.timer_callback)

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
        positions[idx_f] = 1.57 #0.78  # +45°

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
