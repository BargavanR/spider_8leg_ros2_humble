#!/usr/bin/env python3
import sys
import subprocess
import threading
import signal
import psutil
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QGridLayout)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class RobotJoystickGUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, "robot_joystick_gui")
        QWidget.__init__(self)

        self.current_process = None
        self.is_running = False

        # ROS2 publisher for stop command
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/position_controller/joint_trajectory',
            10
        )

        # Joint names for stop command
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

        # Scripts mapping
        self.scripts = {
            'Forward': 'Forward_perfect.py',
            'Backward': 'Backward_perfect.py',
            'Left Turn': 'Left_Turn.py',
            'Right Turn': 'Right_Turn.py',
            'Circle CW': 'Circle_C.py',
            'Circle CCW': 'Circle_CC.py',
            'Dance': 'Dancer_spider.py',
            'Jolly Mode': 'Jolly_mode_play.py'
        }

        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # Title
        title = QLabel("🕷️ Spider Robot Control 🕷️")
        title_font = QFont("Arial", 20, QFont.Bold)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        # Status label
        self.status_label = QLabel("Status: STOPPED")
        status_font = QFont("Arial", 14)
        self.status_label.setFont(status_font)
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("background-color: #ff6b6b; padding: 10px; border-radius: 5px;")
        main_layout.addWidget(self.status_label)

        # Joystick grid layout
        grid = QGridLayout()
        grid.setSpacing(10)

        # Movement buttons in joystick pattern
        # Row 0: Forward
        self.create_button(grid, 'Forward', 0, 1, "⬆️")
        
        # Row 1: Left, Stop, Right
        self.create_button(grid, 'Left Turn', 1, 0, "⬅️")
        self.stop_btn = QPushButton("⏹️ STOP")
        self.stop_btn.setMinimumSize(120, 80)
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #ff4444;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #cc0000;
            }
        """)
        self.stop_btn.clicked.connect(self.stop_robot)
        grid.addWidget(self.stop_btn, 1, 1)
        
        self.create_button(grid, 'Right Turn', 1, 2, "➡️")
        
        # Row 2: Backward
        self.create_button(grid, 'Backward', 2, 1, "⬇️")

        main_layout.addLayout(grid)

        # Special moves section
        special_layout = QVBoxLayout()
        special_label = QLabel("Special Moves")
        special_label.setFont(QFont("Arial", 12, QFont.Bold))
        special_label.setAlignment(Qt.AlignCenter)
        special_layout.addWidget(special_label)

        special_grid = QGridLayout()
        self.create_button(special_grid, 'Circle CW', 0, 0, "🔄")
        self.create_button(special_grid, 'Circle CCW', 0, 1, "🔃")
        self.create_button(special_grid, 'Dance', 1, 0, "💃")
        self.create_button(special_grid, 'Jolly Mode', 1, 1, "🎪")
        
        special_layout.addLayout(special_grid)
        main_layout.addLayout(special_layout)

        self.setLayout(main_layout)
        self.setWindowTitle("Spider Robot Joystick Control")
        self.resize(500, 600)

    def create_button(self, layout, name, row, col, emoji):
        """Create a styled button and add to layout"""
        btn = QPushButton(f"{emoji}\n{name}")
        btn.setMinimumSize(120, 80)
        btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 14px;
                font-weight: bold;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
        """)
        btn.clicked.connect(lambda: self.run_script(name))
        layout.addWidget(btn, row, col)

    def run_script(self, script_name):
        """Run the selected movement script"""
        if self.is_running:
            self.stop_robot()
        
        script_file = self.scripts.get(script_name)
        if not script_file:
            self.get_logger().error(f"Script not found: {script_name}")
            return

        try:
            # Run the script using ros2 run
            command = ['ros2', 'run', 'spider_control', script_file]
            self.current_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.is_running = True
            self.status_label.setText(f"Status: RUNNING - {script_name}")
            self.status_label.setStyleSheet("background-color: #4CAF50; padding: 10px; border-radius: 5px;")
            self.get_logger().info(f"Started: {script_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to run {script_name}: {str(e)}")
            self.status_label.setText(f"Status: ERROR - {str(e)}")
            self.status_label.setStyleSheet("background-color: #ff9800; padding: 10px; border-radius: 5px;")

    def stop_robot(self):
        """Stop the currently running script and send neutral position"""
        # Kill the running process and all its children
        if self.current_process:
            try:
                # Get the process
                parent = psutil.Process(self.current_process.pid)
                
                # Kill all child processes first
                children = parent.children(recursive=True)
                for child in children:
                    try:
                        child.kill()
                        self.get_logger().info(f"Killed child process: {child.pid}")
                    except psutil.NoSuchProcess:
                        pass
                
                # Kill the parent process
                parent.kill()
                parent.wait(timeout=3)
                self.get_logger().info(f"Killed parent process: {parent.pid}")
                
            except psutil.NoSuchProcess:
                self.get_logger().warn("Process already terminated")
            except Exception as e:
                self.get_logger().error(f"Error killing process: {str(e)}")
                # Force kill as last resort
                try:
                    self.current_process.kill()
                except:
                    pass
            
            self.current_process = None

        # Send neutral position command
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        # All joints to neutral (0.0)
        point.positions = [0.0] * len(self.joint_names)
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        msg.points.append(point)
        
        self.publisher.publish(msg)
        self.get_logger().info("Sent neutral position command")

        self.is_running = False
        self.status_label.setText("Status: STOPPED")
        self.status_label.setStyleSheet("background-color: #ff6b6b; padding: 10px; border-radius: 5px;")

    def closeEvent(self, event):
        """Handle window close event"""
        self.stop_robot()
        event.accept()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    
    gui = RobotJoystickGUI()
    gui.show()

    # Run ROS2 spin in separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    ros_thread.start()

    exit_code = app.exec_()

    # Cleanup
    gui.stop_robot()
    gui.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == "__main__":
    main()