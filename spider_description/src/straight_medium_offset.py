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

        # Alternating tripod groups for stability
        self.group_a = [0, 2, 4, 6]  # Legs 1,3,5,7
        self.group_b = [1, 3, 5, 7]  # Legs 2,4,6,8
        
        self.current_group = 'A'
        self.phase = 0
        
        # Gait parameters
        self.coxa_forward = 0.35      # Reduced for more control
        self.coxa_backward = -0.35    
        self.femur_lift = 0.45
        self.femur_stance = 0.0       # Flat on ground
        
        # Variable timing for different phases
        self.phase_durations = [0.25, 0.15, 0.30, 0.15]  # lift, place, push, reset
        self.current_phase_duration = self.phase_durations[0]
        
        self.timer = self.create_timer(self.current_phase_duration, self.timer_callback)
        self.get_logger().info('Starting corrected tripod gait with reset phase')

    def update_timer(self, duration):
        """Update timer with new duration"""
        self.timer.cancel()
        self.current_phase_duration = duration
        self.timer = self.create_timer(duration, self.timer_callback)

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)

        swing_legs = self.group_a if self.current_group == 'A' else self.group_b
        stance_legs = self.group_b if self.current_group == 'A' else self.group_a

        if self.phase == 0:
            # PHASE 0: Lift swing group and move forward
            for leg_idx in swing_legs:
                coxa_idx = leg_idx * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.coxa_forward
                positions[femur_idx] = self.femur_lift
            
            for leg_idx in stance_legs:
                coxa_idx = leg_idx * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.coxa_backward  # Already pushed back
                positions[femur_idx] = self.femur_stance
                
            self.get_logger().info(f'Phase 0: Lift & forward - Group {self.current_group}')
            
        elif self.phase == 1:
            # PHASE 1: Place swing legs down
            for leg_idx in swing_legs:
                coxa_idx = leg_idx * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.coxa_forward
                positions[femur_idx] = self.femur_stance  # Lower to ground
            
            for leg_idx in stance_legs:
                coxa_idx = leg_idx * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.coxa_backward
                positions[femur_idx] = self.femur_stance
                
            self.get_logger().info(f'Phase 1: Place down - Group {self.current_group}')
            
        elif self.phase == 2:
            # PHASE 2: ALL legs push backward (propulsion)
            for leg_idx in range(8):
                coxa_idx = leg_idx * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.coxa_backward
                positions[femur_idx] = self.femur_stance
                
            self.get_logger().info('Phase 2: ALL push backward')
            
        else:  # phase == 3
            # PHASE 3: RESET stance legs to neutral (prevents drift!)
            for leg_idx in stance_legs:
                coxa_idx = leg_idx * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = 0.0  # RESET to center
                positions[femur_idx] = self.femur_stance
            
            for leg_idx in swing_legs:
                coxa_idx = leg_idx * 3
                femur_idx = coxa_idx + 1
                positions[coxa_idx] = self.coxa_backward  # Stay in position
                positions[femur_idx] = self.femur_stance
                
            self.get_logger().info(f'Phase 3: Reset stance - Group {self.current_group}')

        # Set timing for this movement
        duration_ns = int(self.current_phase_duration * 1_000_000_000)
        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = duration_ns
        traj.points = [point]
        self.pub.publish(traj)

        # Advance to next phase
        self.phase += 1
        if self.phase > 3:
            self.phase = 0
            # Switch groups after full cycle
            self.current_group = 'B' if self.current_group == 'A' else 'A'
        
        # Update timer for next phase
        self.update_timer(self.phase_durations[self.phase])


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