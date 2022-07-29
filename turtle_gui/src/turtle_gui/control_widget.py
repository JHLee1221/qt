#!/usr/bin/env python3

import os

from ament_index_python.resources import get_resource
from geometry_msgs.msg import Twist
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut
from python_qt_binding.QtWidgets import QWidget

import rclpy
from rclpy.qos import QoSProfile

class ControlWidget(QWidget):

    def __init__(self, node):
        super(ControlWidget, self).__init__()
        self.setObjectName('ControlWidget')

        self.node = node

        self.REDRAW_INTERVAL = 30
        self.PUBLISH_INTERVAL = 100
        self.CMD_VEL_X_FACTOR = 1000.0
        self.CMD_VEL_YAW_FACTOR = -10.0

        pkg_name = 'turtle_gui'
        ui_filename = 'turtle_gui.ui'
        topic_name = 'cmd_vel'

        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)

        self.pub_velocity = Twist()
        self.pub_velocity.linear.x = 0.0
        self.pub_velocity.angular.z = 0.0
        self.sub_velocity = Twist()
        self.sub_velocity.linear.x = 0.0
        self.sub_velocity.angular.z = 0.0

        self.lcd_ms.display(0.0)
        self.lcd_rad.display(0.0)

        qos = QoSProfile(depth=10)
        self.publisher = self.node.create_publisher(Twist, topic_name, qos)
        self.subscriber = self.node.create_subscription(Twist, topic_name, self.get_velocity, qos)

        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self.send_velocity)
        self.publish_timer.start(self.PUBLISH_INTERVAL)

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_indicators)
        self.update_timer.start(self.REDRAW_INTERVAL)

        self.btn_up.pressed.connect(self.increase_linear_x)
        self.btn_down.pressed.connect(self.decrease_linear_x)
        self.btn_left.pressed.connect(self.increase_angular_z)
        self.btn_right.pressed.connect(self.decrease_angular_z)
        self.btn_stop.pressed.connect(self.set_stop)

        self.btn_up.setShortcut('w')
        self.btn_down.setShortcut('x')
        self.btn_left.setShortcut('a')
        self.btn_right.setShortcut('d')
        self.btn_stop.setShortcut('s')

        self.shortcut_space = QShortcut(QKeySequence(Qt.Key_Space), self)
        self.shortcut_space.setContext(Qt.ApplicationShortcut)
        self.shortcut_space.activated.connect(self.btn_stop.pressed)

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
        self.lcd_ms.display(self.sub_velocity.linear.x)
        self.lcd_rad.display(self.sub_velocity.angular.z)

    def shutdown_widget(self):
        self.update_timer.stop()
        self.publish_timer.stop()
        self.node.destroy_subscription(self.subscriber)
        self.node.destroy_publisher(self.publisher)
