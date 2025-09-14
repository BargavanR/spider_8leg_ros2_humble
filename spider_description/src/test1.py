#!/usr/bin/env python3

# Import ROS 2 Python client library
import rclpy
from rclpy.node import Node

# Import message type for sending joint trajectory commands
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Import time for adding delays between joint movements
import time


# Define a node class that sequentially rotates robot joints
class SequentialJointRotator(Node):
    def __init__(self):
        # Initialize the node with name "sequential_joint_rotator"
        super().__init__('sequential_joint_rotator')

        # Create a publisher to send JointTrajectory messages
        # Topic: /position_controller/joint_trajectory
        # Queue size: 10
        self.pub = self.create_publisher(
            JointTrajectory,
            '/position_controller/joint_trajectory',
            10
        )

        # List of all joint names for an 8-legged robot (each leg has 3 joints)
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


    # Function to rotate each joint one by one
    def rotate_joints_sequentially(self):
        # Loop through every joint in the robot
        for joint in self.joint_names:
            # Create a new JointTrajectory message
            traj = JointTrajectory()
            traj.joint_names = self.joint_names

            # Create a point that defines joint positions at a given time
            point = JointTrajectoryPoint()

            # Start with all joints at 0.0 rad
            positions = [0.0 for _ in self.joint_names]

            # Find index of the current joint to move
            index = self.joint_names.index(joint)

            # Rotate the current joint to 0.8 rad
            positions[index] = 0.8

            # Assign positions to the point
            point.positions = positions

            # Movement should complete within 2 seconds
            point.time_from_start.sec = 2

            # Add this point to trajectory
            traj.points = [point]

            # Log and publish the command
            self.get_logger().info(f'Rotating joint: {joint} to position 0.8 rad')
            self.pub.publish(traj)

            # Wait 3 seconds to observe motion
            time.sleep(3)

            # Reset the joint back to 0.0 before moving to next joint
            positions[index] = 0.0
            point.positions = positions
            traj.points = [point]

            # Publish reset command
            self.pub.publish(traj)
            self.get_logger().info(f'Resetting joint: {joint} to position 0.0 rad')

            # Wait 2 seconds before going to next joint
            time.sleep(2)


# Main function to start the node
def main(args=None):
    # Initialize ROS 2 Python client
    rclpy.init(args=args)

    # Create the node object
    node = SequentialJointRotator()

    try:
        # Run the sequential joint rotation function
        node.rotate_joints_sequentially()
    except KeyboardInterrupt:
        # Allow graceful shutdown when Ctrl+C is pressed
        pass

    # Shutdown ROS 2 when finished
    rclpy.shutdown()


# Entry point of the script
if __name__ == '__main__':
    main()
