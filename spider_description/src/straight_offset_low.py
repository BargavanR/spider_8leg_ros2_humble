#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SequentialGaitPublisher(Node):
    def __init__(self):
        super().__init__('sequential_gait_publisher')
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

        # ALL legs move same direction for forward motion
        # Use alternating tripod gait for stability
        
        # Group A: legs 1,3,5,7 (alternating pattern)
        # Group B: legs 2,4,6,8 (alternating pattern)
        self.group_a = [0, 2, 4, 6]  # Legs 1,3,5,7
        self.group_b = [1, 3, 5, 7]  # Legs 2,4,6,8
        
        self.current_group = 'A'
        self.phase = 0  # 0=lift+forward, 1=lower, 2=all push back
        
        # ALL POSITIVE for forward motion
        self.coxa_forward = -0.4     # Swing forward angle
        self.coxa_backward = 0.4   # Push back angle  
        self.femur_lift = 0.5       # Lift height
        
        self.timer = self.create_timer(0.25, self.timer_callback)
        self.get_logger().info('Starting tripod gait - ALL coxa same direction')

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)

        if self.phase == 0:
            # PHASE 0: Lift one group and swing forward
            swing_legs = self.group_a if self.current_group == 'A' else self.group_b
            stance_legs = self.group_b if self.current_group == 'A' else self.group_a
            
            for leg_idx in swing_legs:
                coxa_idx = leg_idx * 3
                femur_idx = coxa_idx + 1
                # ALL POSITIVE - swing forward while lifted
                positions[coxa_idx] = self.coxa_forward
                positions[femur_idx] = self.femur_lift
            
            for leg_idx in stance_legs:
                coxa_idx = leg_idx * 3
                femur_idx = coxa_idx + 1
                # Stay in place, on ground
                positions[coxa_idx] = 0.0
                positions[femur_idx] = 0.0
                
            self.get_logger().info(f'Phase 0: Lifting group {self.current_group}')
            
        elif self.phase == 1:
            # PHASE 1: Lower the swung legs
            swing_legs = self.group_a if self.current_group == 'A' else self.group_b
            stance_legs = self.group_b if self.current_group == 'A' else self.group_a
            
            for leg_idx in swing_legs:
                coxa_idx = leg_idx * 3
                femur_idx = coxa_idx + 1
                # Keep forward position, lower down
                positions[coxa_idx] = self.coxa_forward
                positions[femur_idx] = 0.0
            
            for leg_idx in stance_legs:
                coxa_idx = leg_idx * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = 0.0
                positions[femur_idx] = 0.0
                
            self.get_logger().info(f'Phase 1: Lowering group {self.current_group}')
            
        else:  # phase == 2
            # PHASE 2: ALL legs push backward together (propulsion)
            for leg_idx in range(8):
                coxa_idx = leg_idx * 3
                femur_idx = coxa_idx + 1
                # ALL NEGATIVE - push backward (robot moves forward)
                positions[coxa_idx] = self.coxa_backward
                positions[femur_idx] = 0.0
                
            self.get_logger().info('Phase 2: ALL legs pushing backward')

        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 250_000_000
        traj.points = [point]
        self.pub.publish(traj)

        # Advance phase
        self.phase += 1
        if self.phase > 2:
            self.phase = 0
            # Switch groups
            self.current_group = 'B' if self.current_group == 'A' else 'A'


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