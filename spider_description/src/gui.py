#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk

class JointControlGUI(Node):
    def __init__(self):
        super().__init__('joint_control_gui')

        # Publisher to send JointTrajectory messages to ros2_control position_controller
        self._trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/position_controller/joint_trajectory',
            10
        )

        # List of joints as defined in your controller YAML
        self.joints = [
            'joint1_coxa', 'joint1_fumer', 'joint1_tibia',
            'joint2_coxa', 'joint2_fumer', 'joint2_tibia',
            'joint3_coxa', 'joint3_fumer', 'joint3_tibia',
            'joint4_coxa', 'joint4_fumer', 'joint4_tibia',
            'joint5_coxa', 'joint5_fumer', 'joint5_tibia',
            'joint6_coxa', 'joint6_fumer', 'joint6_tibia',
            'joint7_coxa', 'joint7_fumer', 'joint7_tibia',
            'joint8_coxa', 'joint8_fumer', 'joint8_tibia'
        ]

        # Store current target positions from sliders
        self.joint_positions = {joint: 0.0 for joint in self.joints}

        # Build Tkinter GUI
        self.window = tk.Tk()
        self.window.title("Spider Robot Joint Controller")

        self.sliders = {}
        for i, joint in enumerate(self.joints):
            label = tk.Label(self.window, text=joint)
            label.grid(row=i, column=0)
            slider = tk.Scale(self.window, from_=-3.14, to=3.14, resolution=0.01,
                              orient=tk.HORIZONTAL, length=300,
                              command=lambda val, j=joint: self.update_joint_position(j, float(val)))
            slider.grid(row=i, column=1)
            self.sliders[joint] = slider

        send_button = tk.Button(self.window, text="Send Trajectory", command=self.send_trajectory_command)
        send_button.grid(row=len(self.joints), column=0, columnspan=2, pady=10)

    def update_joint_position(self, joint, value):
        """Update stored joint position from slider callback."""
        self.joint_positions[joint] = value

    def send_trajectory_command(self):
        """Publish a JointTrajectory message with current slider positions."""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joints

        point = JointTrajectoryPoint()
        point.positions = [self.joint_positions[joint] for joint in self.joints]
        point.time_from_start.sec = 1  # Command trajectory duration: 1 second

        traj_msg.points = [point]

        self._trajectory_pub.publish(traj_msg)
        self.get_logger().info("Published JointTrajectory command")

    def run(self):
        """Run the Tkinter main loop."""
        self.window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    gui_node = JointControlGUI()

    # Spin rclpy in background thread to handle ROS2 communication
    import threading
    def spin():
        rclpy.spin(gui_node)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    # Run the Tkinter GUI loop (blocking)
    gui_node.run()

    gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
