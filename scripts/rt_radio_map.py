#!/usr/bin/env python3
"""
Scrip to read the radio signals and plot then in a array of markers in RVIZ

version 2.0 -> Adding possibility to use dbm instead of the RSSI metric

"""

import rospy

import sys 
import select 
import termios
import tty
import math 
import tf
# import message_filters

from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray
from radio_lib import RadioLib



class RadioSignal:
    def __init__(self):

        # Starting variables
        self.is_dbm = True # Decide if will catch the dbm or rssi radio signal


        # Class variables
        self.radio_RSSI = 0  # Instant radio signal intensity
        self.x = 0  # Instant robot position (x)
        self.y = 0

        self.first_time = True

        self.points = [[0, 0]]  # Array containing robot positions
        self.RSSI = []  # Array containing intensities off radio signal

        self.distance_between_signals = False

        # Parameters variables
        self.radio_signal_acquisition_method = 1
        self.time_interval = 0.1
        self.distance_interval = 0.2

        # Marker variables
        self.radio_markers_array = MarkerArray()
        self.radio_markers_array.markers = []

        self.radio_new_marker = Marker()
        self.marker_counter = 1
        self.marker_size = 0.8
        self.marker_frame_id = 'map'

        # Starting ROS
        rospy.init_node('radio_to_rviz', anonymous=True)
        rospy.loginfo("Starting radio signal mapping node...")

        self.subscriber_radio = rospy.Subscriber("/radio/status", Float32MultiArray, self.subscriber_radio_callback)
        self.listener = tf.TransformListener()

        self.radio_markers_publisher = rospy.Publisher('/radio/collected_data', MarkerArray, queue_size=10, latch=True)

        self.setting = termios.tcgetattr(sys.stdin)
        self.key = 0
        self.init_node()

    def init_node(self):

        """
        Will initialize the node depending on the signal acquisition method:
        1 - get the signal time in time
        2 - get the signal every x distance
        3 - get the signal every time the up arrow is pressed
        """

        if self.radio_signal_acquisition_method == 1:
            self.get_radio_signal_time()
        elif self.radio_signal_acquisition_method == 2:
            self.get_radio_signal_distance()
        elif self.radio_signal_acquisition_method == 3:
            self.get_radio_signal_keyboard()
        else:
            rospy.logerr(" Error: Incorrect method of signal acquisition \n "
                         "Check /espeleo_radio/config/param.yaml and set radio_signal_acquisition_method correct:"
                         "\n 1 - Get the radio signal every X seconds;"
                         "\n 2 - Get the radio signal every X meters;"
                         "\n 3 - Get the radio signal every time the up arrow is pressed")

    def subscriber_radio_callback(self, message):
        if self.is_dbm:
            self.radio_RSSI = message.data[1]
        else:
            self.radio_RSSI = message.data[0]

    def get_radio_signal_keyboard(self):
        rospy.loginfo("Using Keyboar method to collect radio signal...")
        rospy.loginfo("Use up arrow to collect radio signal.")
        while not rospy.is_shutdown():

            self.key = self.getch(0.5)
            if self.key == '\x1b[A':
                self.RSSI.append(self.radio_RSSI)
                self.points.append([self.x, self.y])
                print("Data collected!")
                self.publish_radio_signal_array()

            elif self.key == '\x03':
                break

    def get_radio_signal_time(self):
        rospy.loginfo("Using time method to collect radio signal...")
        rospy.loginfo("Collecting radio signal every %s seconds.", self.time_interval)

        rate = rospy.Duration(self.time_interval)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform('/map', '/base_link_2d', rospy.Time(0))
                self.x = trans[0]
                self.y = trans[1]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.get_radio_signal_distance()
            if self.distance_between_signals:
                self.RSSI.append(self.radio_RSSI)
                self.points.append([self.x, self.y])
                if self.first_time:
                    self.RSSI.pop(0)
                    self.points.pop(0)
                    self.first_time = False
                else:
                    self.publish_radio_signal_array()
            rospy.sleep(rate)

            # TODO -> Utilizar distancia percorrida para nao escrever pontos em cima do outro

    def get_radio_signal_distance(self):

        dist = math.sqrt((self.x - self.points[-1][0]) ** 2 + (self.y - self.points[-1][1]) ** 2)
        if dist < self.distance_interval:
            self.distance_between_signals = False
        else:
            self.distance_between_signals = True

    def publish_radio_signal_array(self):
        #intensity = self.get_rssi_to_rgb(self.RSSI[-1])
        intensity = radio_lib.get_rgb_from_rssi(self.RSSI[-1], self.is_dbm)

        self.radio_new_marker = Marker()
        self.radio_new_marker.header.frame_id = self.marker_frame_id
        self.radio_new_marker.header.stamp = rospy.Time.now()
        self.radio_new_marker.ns = "espeleo_radio_map"
        self.radio_new_marker.id = self.marker_counter
        self.radio_new_marker.type = Marker.CUBE
        self.radio_new_marker.action = Marker.ADD
        self.radio_new_marker.pose.orientation.w = 1.0
        self.radio_new_marker.pose.position.x = self.points[-1][0]
        self.radio_new_marker.pose.position.y = self.points[-1][1]
        self.radio_new_marker.pose.position.z = 0
        self.radio_new_marker.scale.x = self.marker_size
        self.radio_new_marker.scale.y = self.marker_size
        self.radio_new_marker.scale.z = 0.1

        self.radio_new_marker.color.a = 1.0
        self.radio_new_marker.color.b = 1 - 1.0 * intensity
        self.radio_new_marker.color.r = 1.0 * intensity
        self.radio_new_marker.color.g = 0.8 * (1 - 2.0 * abs(intensity - 1 / 2))
        self.marker_counter += 1
        self.radio_new_marker.lifetime = rospy.Duration(0)

        self.radio_markers_array.markers.append(self.radio_new_marker)
        self.radio_markers_publisher.publish(self.radio_markers_array)

    def getch(self, timeout=None):
        self.settings = termios.tcgetattr(sys.stdin)
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


if __name__ == '__main__':
    radio_lib = RadioLib()
    radio_obj = RadioSignal()
    