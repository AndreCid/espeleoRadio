#!/usr/bin/env python

from math import sin, cos, pi, log10, sqrt


import rospy
import tf
import math

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import openturns as ot

import numpy as np
import sys, select, termios, tty

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from scipy.interpolate import griddata
from PIL import Image




class radio_signal:
    def __init__(self):

        # Starting variables

        #  Interpolation variables
        self.radio_RSSI = 0
        self.x = 0
        self.y = 0
        self.interpolation_method = 0

        self.points = []
        self.RSSI = []

        self.interpolation_counter = 0


        # Plotting variables
        self.path = '/home/andre/catkin_ws/src/espeleo_2dnav/maps/mina_coppelia.pgm'
        self.map_origin = rospy.get_param("origin") # Coordinates in meters of the lowest left pixel
        self.map_resolution = rospy.get_param("resolution") # meters/pixel
        self.point_to_plot_y = [0]
        self.point_to_plot_x = [0]
        self.map_points = []

        # Starting ROS
        rospy.init_node('radio_mapper', anonymous=True)
        rospy.loginfo("Starting radio singnal mapping node...")
        rospy.loginfo("Use up arrow to collect radio signal.")
        self.subscriber_radio = rospy.Subscriber("/radio/status", Float32MultiArray, self.subscriber_radio_callback)
        self.subscriber_odom = rospy.Subscriber("/tf", TFMessage, self.odom_callback)

        self.setting = termios.tcgetattr(sys.stdin)
        self.key = 0
        self.moveBindings = {'\x1b[A': (1, 0, 0, 0)}
        self.init_node()



    def init_node(self):
        while not rospy.is_shutdown():
            self.key = self.getch(0.5)
            if self.key == '\x1b[A':
                self.RSSI.append(self.radio_RSSI)
                self.points.append((self.x, self.y))
                print("Data collected")


            if self.key == '\x1b[B':
                self.print_map()

            elif (self.key == '\x03'):
                break

    def subscriber_radio_callback(self, message):
        self.radio_RSSI = message.data[0]

    def odom_callback(self, message):
        for T in message.transforms:
            # Choose the transform of the EspeleoRobo
            if (T.child_frame_id == "base_link"):
                # Get the position
                self.x = T.transform.translation.x
                self.y = T.transform.translation.y

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

        """
        try:
            self.interpolate()
        except rospy.ROSInterruptException:
            pass


    def interpolate(self):

        #if self.interpolation_method == 0:
           # print("values", self.RSSI)
           # print("position", self.points)
           # self.linear_interpolation()


    def linear_interpolation(self):
        if self.interpolation_counter <= 10:
            self.interpolation_counter = self.interpolation_counter+1

        else:
            print("coordenadas", len(self.points))
            print("sinais", len(self.RSSI))
            grid_x, grid_y = np.mgrid[0:1:200j, 0:1:200j]
            grid_z1 = griddata(self.points, self.RSSI, (grid_x, grid_y), method='linear')
            plt.imshow(grid_z1.T, extent=(0,1,0,1), origin='lower')
            plt.show()


    #def krieg_interpolation(self):
"""

    def print_map(self):
        map_image = mpimg.imread(self.path)

        #img_show = plt.imshow(map_image, cmap='gray')
        img_show = plt.imshow(np.flipud(map_image), origin='lower,', cmap='gray')

        self.point_to_plot_x = [math.floor(abs(item[0]-self.map_origin[0])/self.map_resolution) for item in self.points]
        self.point_to_plot_y = [(len(map_image[1])-math.floor(abs(item[1]-self.map_origin[0])/self.map_resolution)) for item in self.points]

        self.point_to_plot_y = [
            (math.floor(abs(item[1] - self.map_origin[0]) / self.map_resolution)) for item in
            self.points]

        """

        for i in range(len(self.points)):
            print("opa")
            self.point_to_plot_x[i] = math.floor(abs(self.points[i][0]-self.map_origin[0])/self.map_resolution)
            self.point_to_plot_y[i] = (len(map_image[1]) - math.floor(abs(self.points[i][1] - self.map_origin[1])/self.map_resolution))
       """
        #plt.plot(self.point_to_plot_x, self.point_to_plot_y, 'rx')

        #if len(self.point_to_plot_y) > 4:
        self.interpolate()

        plt.show()
        self.map_points = []


    def interpolate(self):
        map_image = mpimg.imread(self.path)
        map_image2 = mpimg.imread(self.path)
        map_image = np.flip(map_image, axis=0)
        map_image = np.transpose(map_image)
        for i in range(len(self.points)):
            self.map_points.append([self.point_to_plot_x[i], self.point_to_plot_y[i]])

        x_low = int(min(self.point_to_plot_x))
        y_low = int(min(self.point_to_plot_y))
        x_max = int(max(self.point_to_plot_x))
        y_max = int(max(self.point_to_plot_y))


        grid_x, grid_y = np.mgrid[x_low:x_max, y_low:y_max]
        #grid_x, grid_y = np.mgrid[833:1129, 1186:1985]
        grid_z2 = griddata(self.map_points, self.RSSI, (grid_x, grid_y), method='cubic')
        grid_z1 = griddata(self.map_points, self.RSSI, (grid_x, grid_y), method='linear')
        #print(x_low,x_max,y_low,y_max)
        #print(self.points)
        new_map = []
        for i in range(len(grid_z1)):
            for j in range(len(grid_z1[1])):
                if map_image[x_low + i][y_low + j] != 254:
                    grid_z1[i][j] = float("nan")
                    grid_z2[i][j] = float("nan")
        print(len(grid_z1))

        plt.imshow(grid_z1.T, extent=(x_low, x_max, y_low, y_max), origin='lower')
        plt.title('Linear')
        plt.colorbar()
        plt.show()
        img_show = plt.imshow(np.flipud(map_image2), origin='lower', cmap='gray')
        #plt.plot(self.point_to_plot_x, self.point_to_plot_y, 'rx')
        plt.imshow(grid_z2.T, extent=(x_low, x_max, y_low, y_max), origin='lower')
        plt.title('Cubico')
        plt.colorbar()
        plt.show()



if __name__ == '__main__':
    radio_obj = radio_signal()