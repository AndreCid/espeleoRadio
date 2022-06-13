#!/usr/bin/env python

from math import sin, cos, pi, log10, sqrt


import rospy

import matplotlib.pyplot as plt
import sys, select, termios, tty

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3



class odometry:
    def __init__(self):
        self.x_roda = []
        self.y_roda = []
        self.th_roda = []


        self.x_simulador = []
        self.y_simulador = []
        self.simulador = []

        # Starting ROS
        rospy.init_node('odom_plotter', anonymous=True)
        rospy.loginfo("Starting radio singnal mapping node...")
        rospy.loginfo("Use up arrow to collect radio signal.")
        self.subscriber_odom_roda = rospy.Subscriber("/odom", Odometry, self.subscriber_odom_roda)
        self.subscriber_odom_simulador = rospy.Subscriber("/pose", Pose, self.odom_callback)

        self.setting = termios.tcgetattr(sys.stdin)
        self.key = 0
        self.init_node()

    def init_node(self):
        while not rospy.is_shutdown():
            self.key = self.getch(0.5)
            if self.key == '\x1b[B':
                self.print_map()

            elif (self.key == '\x03'):
                break

    def odom_callback(self, message):
        self.x_simulador.append(message.position.x)
        self.y_simulador.append(message.position.y)
                #self.simulador.append([])

    def subscriber_odom_roda(self, message):
        self.x_roda.append(message.pose.pose.position.x)
        self.y_roda.append(message.pose.pose.position.y)

    def getch(self, timeout=None):
        settings = termios.tcgetattr(sys.stdin)
        try:
            self.setup_term(sys.stdin)
            try:
                rw, wl, xl = select.select([sys.stdin], [], [], timeout)
            except select.error:
                return
            if rw:
                key = sys.stdin.read(1)

                if key == '\x1b':
                    key += sys.stdin.read(2)

                return key
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.setting)

    def setup_term(self, fd, when=termios.TCSAFLUSH):
        mode = termios.tcgetattr(fd)
        mode[tty.LFLAG] = mode[tty.LFLAG] & ~(termios.ECHO | termios.ICANON)
        termios.tcsetattr(fd, when, mode)

    def print_map(self):
        plt.xlabel('X(m)')
        plt.ylabel('Y(m)')
        plt.title('Odometria de rodas')
        plt.plot(self.x_roda, self.y_roda, 'r-', label='Odometria das rodas')
        plt.plot(self.x_simulador, self.y_simulador, 'k--', label='Odometria do simulador')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    Odom_plot = odometry()