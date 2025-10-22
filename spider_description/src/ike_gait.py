#!/usr/bin/env python3
"""
*****************************************************************************************
*  Filename:       sequential_leg_gait.py
*  Description:    IK-based sequential gait for 8-legged spider robot
*  Author:         BARGAVAN R - SPIDER TEAM MIT
*****************************************************************************************
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

# ---------------------------
# Robot Parameters
# ---------------------------
L1 = 0.0275   # Coxa length (m)
L2 = 0.055    # Femur length
L3 = 0.08185  # Tibia length
STEP_SIZE = 0.010  # 10 mm forward

# Base positions of legs
base_positions = np.array([
    [0.14757, -0.04705, 0.10935],
    [0.08707, -0.04705, 0.10935],
    [0.02657, -0.04705, 0.10935],
    [-0.03393, -0.04705, 0.10935],
    [0.14757,  0.03994, 0.10935],
    [0.08707,  0.03994, 0.10935],
    [0.02657,  0.03994, 0.10935],
    [-0.03393, 0.03994, 0.10935]
])

# Initial foot positions
foot_init = np.array([
    [0.160, -0.060, 0.030],
    [0.100, -0.060, 0.030],
    [0.040, -0.060, 0.030],
    [-0.020, -0.060, 0.030],
    [0.160,  0.060, 0.030],
    [0.100,  0.060, 0.030],
    [0.040,  0.060, 0.030],
    [-0.020, 0.060, 0.030]
])

# ---------------------------
# IK Function
# ---------------------------
def leg_ik(foot_target, base_pos, L1, L2, L3):
    dx = foot_target[0] - base_pos[0]
    dy = foot_target[1] - base_pos[1]
    dz = foot_target[2] - base_pos[2]

    # Coxa
    v1 = np.arctan2(dy, dx)

    # Planar distances
    r = np.sqrt(dx**2 + dy**2) - L1
    s = dz
    D = np.sqrt(r**2 + s**2)
    D = min(D, L2 + L3 - 1e-6)

    # Tibia
    cos_v3 = (L2**2 + L3**2 - D**2) / (2*L2*L3)
    v3 = np.arccos(np.clip(cos_v3, -1, 1)) - np.pi/2

    # Femur
    alpha = np.arctan2(s, r)
    beta  = np.arccos(np.clip((L2**2 + D**2 - L3**2)/(2*L2*D), -1, 1))
    v2 = alpha + beta

    return v1, v2, v3

# ---------------------------
# ROS Node
# ---------------------------
class SequentialLegGait(Node):
    def __init__(self):
        super().__init__('sequential_leg_gait')
        self.pub = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)

        # Joint names
        self.joint_names = []
        for i in range(8):
            self.joint_names += [f'joint{i+1}_coxa', f'joint{i+1}_fumer', f'joint{i+1}_tibia']

        # Leg pairs: 1→5, 2→6, 3→7, 4→8
        self.leg_groups = [(0, 1,6,7), (2, 3,4,5)]
        self.current_step = 0

        # Step size in X (10 mm forward)
        self.step_size = 0.002

        # Timer for sequential gait
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz

    def timer_callback(self):
        # traj = JointTrajectory()
        # traj.joint_names = self.joint_names
        # point = JointTrajectoryPoint()
        # joint_positions = [0.0]*24  # all joints start at zero

        # # Current leg pair
        # leg1, leg2 ,leg3,leg4 = self.leg_pairs[self.current_step]

        # # Compute IK for first leg
        # foot_target1 = foot_init[leg1].copy()
        # foot_target1[1] -= self.step_size
        # coxa1, femur1, tibia1 = leg_ik(foot_target1, base_positions[leg1], L1, L2, L3)
        # foot_target2 = foot_init[leg2].copy()
        # foot_target2[1] -= self.step_size
        # coxa2, femur2, tibia2 = leg_ik(foot_target1, base_positions[leg2], L1, L2, L3)

        # # Compute IK for second leg
        # foot_target3 = foot_init[leg3].copy()
        # foot_target3[1] += self.step_size
        # coxa3, femur3, tibia3 = leg_ik(foot_target2, base_positions[leg3], L1, L2, L3)

        # # Update joint positions
        # joint_positions[leg1*3]   = -coxa1
        # joint_positions[leg1*3+1] = -femur1
        # joint_positions[leg1*3+2] = -tibia1

        # joint_positions[leg2*3]   = -coxa2
        # joint_positions[leg2*3+1] = -femur2
        # joint_positions[leg2*3+2] = -tibia2

        # # Fill message
        # point.positions = joint_positions
        # point.time_from_start.sec = 0
        # point.time_from_start.nanosec = int(0.5*1e9)
        # traj.points = [point]

        # # Publish
        # self.pub.publish(traj)
        # self.get_logger().info(f"Moved leg pair {leg1+1} & {leg2+1}")

        # # Next pair
        # self.current_step = (self.current_step + 1) % len(self.leg_pairs)
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        joint_positions = [0.0]*24  # all joints start at zero

        # Current group
        g1, g2, g3, g4 = self.leg_groups[self.current_step]

        # Move front pair forward
        foot_targets = [
            foot_init[g1].copy(),
            foot_init[g2].copy(),
            foot_init[g3].copy(),
            foot_init[g4].copy()
        ]
            ### make [0][2]  2 in all second for dancee 
            ### 1 for rotational 
            
        foot_targets[0][1] -= STEP_SIZE  # g1 forward X
        foot_targets[1][1] -= STEP_SIZE  # g2 forward X
        foot_targets[2][1] += STEP_SIZE*2  # g3 backward X
        foot_targets[3][1] += STEP_SIZE*2 # g4 backward X

        # Compute IK and assign
        legs = [g1,g2,g3,g4]
        for i, leg in enumerate(legs):
            coxa, femur, tibia = leg_ik(foot_targets[i], base_positions[leg], L1, L2, L3)
            joint_positions[leg*3]   = -coxa
            joint_positions[leg*3+1] = -femur
            joint_positions[leg*3+2] = -tibia

        # Publish
        point.positions = joint_positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.5*1e9)
        traj.points = [point]
        self.pub.publish(traj)
        self.get_logger().info(f"Moved legs {g1+1},{g2+1},{g3+1},{g4+1}")

        # Next group
        self.current_step = (self.current_step + 1) % len(self.leg_groups)

# ---------------------------
# Main
# ---------------------------
def main(args=None):
    rclpy.init(args=args)
    node = SequentialLegGait()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
