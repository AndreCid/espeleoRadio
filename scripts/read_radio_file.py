#!/usr/bin/env python3
"""
Scrip to read a file containing radio signals and plot then in a array of markers in RVIZ

"""

import rospy
import sys
from visualization_msgs.msg import Marker, MarkerArray
from radio_lib import RadioLib


class RadioSignal:
    def __init__(self):

        # Starting variables
        self.self_path = sys.argv[1]
        print(self.self_path)
        # Class variables

        self.RSSI = []
        self.x = []
        self.y = []
        self.max = 0
        self.min = 0

        self.is_dbm = True

        # Marker variables
        self.radio_markers_array = MarkerArray()
        self.radio_markers_array.markers = []

        self.radio_new_marker = Marker()
        self.marker_counter = 1
        self.markers_size = 1
        self.marker_frame_id = 'map'

        # Starting ROS
        rospy.init_node('file_radio_to_rviz', anonymous=True)
        rospy.loginfo("Starting file reading node...")

        self.radio_markers_publisher = rospy.Publisher('/radio/collected_data', MarkerArray, queue_size=10, latch=True)
        self.init_node()

    def init_node(self):
        self.txt_to_data()
        self.publish_radio_signal_array()
        while not rospy.is_shutdown():
            rospy.sleep(rospy.Duration(1))

    def txt_to_data(self):
        file = open(self.self_path)
        file = file.read()
        data = file.split('\n')
        for i in range(len(data)-1):
            try:
                line_data = data[i].split(',')
                self.RSSI.append(float(line_data[0]))
                self.x.append(float(line_data[1]))
                self.y.append(float(line_data[2]))
            except Exception as error:
                rospy.loginfo(error)
                rospy.logwarn('The file should be formatted as: \n'
                              'radio_data_1,x_position_1,y_position_1\n'
                              'radio_data_2,x_position_2,y_position_3\n...00')
        print(radio_obj.get_rgb_from_rssi(min(self.RSSI),  self.is_dbm))
        print(radio_obj.get_rgb_from_rssi(max(self.RSSI),  self.is_dbm))
        print(min(self.RSSI))
        print(max(self.RSSI))
        rospy.loginfo("File readied! Ploting points...")

    def publish_radio_signal_array(self):

        for i in range(len(self.RSSI)):
            intensity = radio_obj.get_rgb_from_rssi(self.RSSI[i], self.is_dbm)
            self.radio_new_marker = Marker()
            self.radio_new_marker.header.frame_id = self.marker_frame_id
            self.radio_new_marker.header.stamp = rospy.Time.now()
            self.radio_new_marker.ns = "espeleo_radio_map"
            self.radio_new_marker.id = self.marker_counter
            self.radio_new_marker.type = Marker.CUBE
            self.radio_new_marker.action = Marker.ADD
            self.radio_new_marker.pose.orientation.w = 1.0
            self.radio_new_marker.pose.position.x = self.x[i]
            self.radio_new_marker.pose.position.y = self.y[i]
            self.radio_new_marker.pose.position.z = 1
            self.radio_new_marker.scale.x = self.markers_size
            self.radio_new_marker.scale.y = self.markers_size
            self.radio_new_marker.scale.z = 0.1

            self.radio_new_marker.color.a = 1.0
            self.radio_new_marker.color.g = 0.8 * (1 - 2.0 * abs(intensity - 1 / 2))
            self.radio_new_marker.color.r = 1.0 * intensity
            self.radio_new_marker.color.b = 1 - 1.0 * intensity
            self.marker_counter += 1
            self.radio_new_marker.lifetime = rospy.Duration(0)

            self.radio_markers_array.markers.append(self.radio_new_marker)
        self.radio_markers_publisher.publish(self.radio_markers_array)


if __name__ == '__main__':
    radio_obj = RadioLib()
    publish_static_data = RadioSignal()
