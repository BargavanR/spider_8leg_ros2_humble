#!/usr/bin/env python3
import sys
import threading
import time
import math
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointGUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, "joint_gui")
        QWidget.__init__(self)

        # 8-legged robot joints
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
        self.positions = [0.0] * len(self.joint_names)

        # ROS2 publisher
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/position_controller/joint_trajectory',
            10
        )

        # PyQt layout
        layout = QVBoxLayout()
        self.labels = []
        self.sliders = []

        for i, name in enumerate(self.joint_names):
            label = QLabel(f"{name}: 0.0")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-100)
            slider.setMaximum(100)
            slider.setValue(0)
            slider.valueChanged.connect(lambda val, j=i, l=label: self.update_label(j, val, l))
            layout.addWidget(label)
            layout.addWidget(slider)
            self.labels.append(label)
            self.sliders.append(slider)

        # Update button (send current positions)
        update_button = QPushButton("Update Joints")
        update_button.clicked.connect(self.publish_trajectory)
        layout.addWidget(update_button)

        # Sine-wave motion button
        sine_button = QPushButton("Sine-Wave Motion")
        sine_button.clicked.connect(self.sine_wave_motion)
        layout.addWidget(sine_button)

        self.setLayout(layout)
        self.setWindowTitle("8-Legged Robot Joint Controller")
        self.resize(400, 700)

    def update_label(self, index, val, label):
        pos = val / 100.0
        self.positions[index] = pos
        label.setText(f"{self.joint_names[index]}: {pos:.2f}")

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.publisher.publish(msg)
        print(f"Published positions: {self.positions}")

    # ---------------- Sine-Wave Motion ----------------
    def sine_wave_motion(self, duration=4.0, steps=80):
        """
        Make all 8 legs follow a sine wave motion from 0 -> -1 -> 0.
        duration : total time for one sine wave cycle (seconds)
        steps    : number of intermediate points for smoothness
        """
        if duration <= 0:
            duration = 4.0
        for i in range(steps + 1):
            t = (i / steps) * duration  # current time in the cycle
            pos = -math.sin(math.pi * t / duration)  # sine wave from 0 -> -1 -> 0
            

            # Apply the same position to all joints
            self.positions = [pos] * len(self.positions)
            
            # Publish the trajectory
            self.publish_trajectory()

            # Small delay for smooth motion
            time.sleep(duration / steps)
def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = JointGUI()
    gui.show()

    ros_thread = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    ros_thread.start()

    sys.exit(app.exec_())

    gui.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
