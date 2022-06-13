#!/usr/bin/env python

"""
Programa para gerar um mapa de calor manual de qualquer lugar necessitando de odometria e sinal de radio.
- Subscriber da odometria e do sinal de radio;
- Quando pressionado a seta para cima do teclado, o programa pega a intensidade do sinal da odometria e salva eles em
uma posicao do vetor do tipo [[radio1,odom1],[radio2,odom2]...[radion,odomn]];
- Salva o vetor em um arquivo txt para posterior analise;
- Seta para baixo gera o mapa

"""



import rospy
import sys
import select
import termios
import tty

import matplotlib.pyplot as plt
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from scipy.interpolate import griddata


class RadioSignal:
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
        # self.map_origin = rospy.get_param("origin") # Coordinates in meters of the lowest left pixel
        # self.map_resolution = rospy.get_param("resolution") # meters/pixel
        self.point_to_plot_y = [0]
        self.point_to_plot_x = [0]
        self.map_points = []

        # Starting ROS
        rospy.init_node('radio_mapper', anonymous=True)
        rospy.loginfo("Starting radio singnal mapping node...")
        rospy.loginfo("Iniciado....")
        self.subscriber_radio = rospy.Subscriber("/radio/status", Float32MultiArray, self.subscriber_radio_callback)
        self.subscriber_odom = rospy.Subscriber("/tf", TFMessage, self.odom_callback)

        self.setting = termios.tcgetattr(sys.stdin)
        self.key = 0
        self.moveBindings = {'\x1b[A': (1, 0, 0, 0)}

        # Creating file
        self.txt_filename = "radio_odom_points_" + str(rospy.Time.now()) + ".txt"
        self.file = open(self.txt_filename, 'w')

        self.init_node()



    def init_node(self):
        while not rospy.is_shutdown():
            #self.key = self.getch(0.5)
            #if self.key == '\x1b[A':
            rospy.loginfo("Preparando para coletar sinal de radio... Aguarde 40 segundos...")
            rospy.sleep(35.)
            self.RSSI.append(self.radio_RSSI)
            self.points.append((self.x, self.y))
            self.file.write(str(self.radio_RSSI) + "," + str(self.x) + "," + str(self.y) + "\n")
            rospy.loginfo("Dado coletado!")
            rospy.loginfo(str(self.radio_RSSI) + "," + str(self.x) + "," + str(self.y))
            rospy.loginfo("Intervalo de movimento - 8 segundos")
            rospy.sleep(8)


            """
            if self.key == '\x1b[B':
                self.print_map()

            elif (self.key == '\x03'):
                break
            """

    def subscriber_radio_callback(self, message):
        self.radio_RSSI = message.data[0]

    def odom_callback(self, message):
        for T in message.transforms:
            # Choose the transform from seekur
            if (T.child_frame_id == "base_link"):
                # Get the position
                self.x = T.transform.translation.x
                self.y = T.transform.translation.y
        # self.x = message.pose.pose.position.x
        # self.y = message.pose.pose.position.y

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

        x_low = int(min(self.point_to_plot_x))
        y_low = int(min(self.point_to_plot_y))
        x_max = int(max(self.point_to_plot_x))
        y_max = int(max(self.point_to_plot_y))

        grid_x, grid_y = np.mgrid[x_low:x_max, y_low:y_max]
        grid_z1 = griddata(self.points, self.RSSI, (grid_x, grid_y), method='linear')
        print(len(grid_z1))

        plt.imshow(grid_z1.T, extent=(x_low, x_max, y_low, y_max), origin='lower')
        plt.title('Linear')
        plt.colorbar()
        plt.show()


if __name__ == '__main__':
    radio_obj = RadioSignal()