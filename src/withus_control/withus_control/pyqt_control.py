import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QPushButton
from PyQt5.QtGui import QColor
from PyQt5.QtCore import Qt, QTimer
import rclpy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
import os
import subprocess

class VelControlApp(QWidget):
    def __init__(self):
        super().__init__()

        self.init_ui()
        self.init_ros()

    def init_ui(self):
        self.setWindowTitle('Cmd Vel and Steering Control')
        self.setGeometry(200, 200, 800, 400)

        self.vel_label = QLabel('Cmd Vel: 0', self)
        self.steering_label = QLabel('Steering Angle: 0', self)

        self.vel_slider = QSlider(Qt.Horizontal, self)
        self.vel_slider.setRange(0, 70)
        self.vel_slider.setValue(0)
        self.vel_slider.valueChanged.connect(self.update_vel_label)

        self.steering_slider = QSlider(Qt.Horizontal, self)
        self.steering_slider.setRange(-30, 30)
        self.steering_slider.setValue(0)
        self.steering_slider.valueChanged.connect(self.update_steering_label)

        self.brake_button = QPushButton('Brake', self)
        self.brake_button.clicked.connect(self.toggle_brake)
        self.close_button = QPushButton('Close', self)
        self.close_button.clicked.connect(self.close_app)

        layout = QVBoxLayout()
        layout.addWidget(self.vel_label)
        layout.addWidget(self.vel_slider)
        layout.addWidget(self.steering_label)
        layout.addWidget(self.steering_slider)
        layout.addWidget(self.brake_button)
        layout.addWidget(self.close_button)

        self.setLayout(layout)

        # Create a QTimer to control the publishing frequency
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.send_continuous_velocities)
        self.timer.start(100)  # Publish every 100 milliseconds (10 Hz)

        # Flag to track brake state
        self.brake_active = False
        self.break_stop = subprocess.Popen(
            ['gnome-terminal', '--disable-factory', '-x', 'ros2', 'param', 'set', 'can_subscriber', 'break', '0'],
            preexec_fn=os.setpgrp)

    def init_ros(self):
        rclpy.init()
        self.node = rclpy.create_node('vel_control_app')

        # Define the brake parameter


        
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update_vel_label(self):
        vel_value = float(self.vel_slider.value())
        self.vel_label.setText(f'Cmd Vel: {vel_value} km/hr')

    def update_steering_label(self):
        steering_value = float(self.steering_slider.value())
        self.steering_label.setText(f'Steering Angle: {steering_value} degrees')

    def toggle_brake(self):
        # Toggle the brake state and set the parameter value
        self.brake_active = not self.brake_active
        if self.brake_active == True:
            self.break_stop = subprocess.Popen(
                ['gnome-terminal', '--disable-factory', '-x', 'ros2', 'param', 'set', 'can_subscriber', 'break', '1'],
                preexec_fn=os.setpgrp
)
        else:
           self.break_stop = subprocess.Popen(
                ['gnome-terminal', '--disable-factory', '-x', 'ros2', 'param', 'set', 'can_subscriber', 'break', '0'],
                preexec_fn=os.setpgrp
)

        # Change the color of the UI based on the brake state
        color = "white" if self.brake_active else "red"
        self.brake_button.setStyleSheet(f"background-color: {color}; color: black;")


    def close_app(self):
        # Set brake parameter to 1 and close the application
   
        self.break_stop = subprocess.Popen(
            ['gnome-terminal', '--disable-factory', '-x', 'ros2', 'param', 'set', 'can_subscriber', 'break', '0'],
            preexec_fn=os.setpgrp)
        self.close()

    def send_continuous_velocities(self):
        # This method is called by the timer to continuously send velocities
        vel_value = float(self.vel_slider.value())
        steering_value = float(self.steering_slider.value())

        # Publish the speed and steering values
        twist_msg = Twist()
        twist_msg.linear.x = vel_value
        twist_msg.angular.z = steering_value
        self.cmd_vel_pub.publish(twist_msg)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    vel_control_app = VelControlApp()
    vel_control_app.show()
    sys.exit(app.exec_())
