#!/usr/bin/env/python3

import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import QoSProfile


ui_path = r"/home/leejeonghun/turtle_ws/src/turtle_gui/resource/turtle_gui.ui"
form_class = uic.loadUiType(ui_path)[0]

class WindowClass(QMainWindow, form_class) :
    def __init__(self, node):
        super(WindowClass, self).__init__()
        self.setupUi(self)
        self.node = node

        self.REDRAW_INTERVAL = 30
        self.PUBLISH_INTERVAL = 100
        self.CMD_VEL_X_FACTOR = 1000.0
        self.CMD_VEL_YAW_FACTOR = -10.0

        topic_name = 'cmd_vel'

        self.pub_velocity = Twist()
        self.pub_velocity.linear.x = 0.0
        self.pub_velocity.angular.z = 0.0
        self.sub_velocity = Twist()
        self.sub_velocity.linear.x = 0.0
        self.sub_velocity.angular.z = 0.0

        qos = QoSProfile(depth=10)
        self.publisher = self.node.create_publisher(Twist, topic_name, qos)
        self.subscriber = self.node.create_subscription(Twist, topic_name, self.get_velocity, qos)
        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self.send_velocity)
        self.publish_timer.start(self.PUBLISH_INTERVAL)

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_indicators)
        self.update_timer.start(self.REDRAW_INTERVAL)

        self.btn_up.clicked.connect(self.increase_linear_x)
        self.btn_stop.clicked.connect(self.set_stop)
        self.btn_left.clicked.connect(self.increase_linear_z)
        self.btn_down.clicked.connect(self.decrease_linear_x)
        self.btn_right.clicked.connect(self.decrease_linear_z)

        self.btn_up.setShortcut('w')
        self.btn_stop.setShortcut('s')
        self.btn_left.setShortcut('a')
        self.btn_down.setShortcut('x')
        self.btn_right.setShortcut('d')

        self.shortcut_space = QShortcut(QKeySequence(Qt.Key_Space), self)
        self.shortcut_space.setContext(Qt.ApplicationShortcut)
        self.shortcut_space.activated.connect(self.push_button_s.pressed)

    def get_velocity(self, msg):
        self.sub_velocity = msg

    def increase_linear_x(self):
        self.pub_velocity.linear.x += 0.1

    def decrease_linear_x(self):
        self.pub_velocity.linear.x -= 0.1

    def increase_angular_z(self):
        self.pub_velocity.angular.z += 0.1

    def decrease_angular_z(self):
        self.pub_velocity.angular.z -= 0.1

    def set_stop(self):
        self.pub_velocity.linear.x = 0.0
        self.pub_velocity.angular.z = 0.0

    def send_velocity(self):
        twist = Twist()
        twist.linear.x = self.pub_velocity.linear.x
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.pub_velocity.angular.z
        self.publisher.publish(twist)

    def update_indicators(self):
        self.dial_yaw.setValue(self.sub_velocity.angular.z * self.CMD_VEL_YAW_FACTOR)
        self.lcd_number_x.display(self.sub_velocity.linear.x)
        self.lcd_number_yaw.display(self.sub_velocity.angular.z)

    def shutdown_widget(self):
        self.update_timer.stop()
        self.publish_timer.stop()
        self.node.destroy_subscription(self.subscriber)
        self.node.destroy_publisher(self.publisher)


if __name__ == "__main__" :
    app = QApplication(sys.argv)
    myWindow = WindowClass(Twist)
    myWindow.show()
    app.exec_()
