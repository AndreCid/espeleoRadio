#!/usr/bin/env python3

"""
Ideia: preditor de radio com vários módos de acordo com aproximação e ambiente
Fórmulas:


Primeiro: Subscriber do mapa 2D a cada 5 segundos ou mais.
"""

import rospy
import time
import math

import matplotlib.pyplot as plt
import numpy as np

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger, Empty
from visualization_msgs.msg import Marker, MarkerArray
from radio_lib import RadioLib


class RadioPredictor:
    def __init__(self):

        # Debug
        self.debug = True

        # Antenna position in X,y meters related to initial robot pose
        self.base_antenna = [-4.32, 6.22]
        self.direct_view_distance = 0
        self.largest_linear_antenna_dimension = 0.8
        self.df = 0

        """
        Transmited max power - Radio Ubiquiti ROcket M900
            Value:  28dBm
            Source: https://dl.ui.com/datasheets/rocketm/RocketM_DS.pdf  -->  pag 13

        Receiver antenna gain - Antenna omnidirectional MU-00PI
            Value:  6dBi
            Source: https://arseletronica.com.br/solucoes/antena-movel-mu-00pi/

        Trasmitted antenna gain - Aquário antenna CF-914
            Value:  14dBi
            Source: https://aquario.com.br/produto/antena-externa-celular-900mhz-14db-cf-914/
        """

        self.Pt_dbm = 28  # Transmitted power in dBm
        self.Gt_dBi = 16  # Gain of the Transmitted antenna in dBi
        self.Gr_dBi = 6  # Gain of the Receiver antenna in dBi
        self.f = 912000000  # frequency of transmitted signal in Hertz
        self.lmbda = (3*(10**8))/self.f
        self.d = 0  # array of distances at which the loss needs to be calculated
        self.L = 1  # Other System Losses, No Loss case L=1
        self.n = 3  # path loss exponent (n=2 for free space)
        self.Pr_dBm = 0  # Received power in dBm
        self.PL_dB = 0  # constant path loss factor (including antenna gains)

        """
        self.reference_point = [-6.59, -2.33]
        self.reference_point = [-15.789640829964053, -2.5294702363692116]
        self.reference_point = self.calculate_distance_between_antenna(
            self.reference_point)
        print(self.reference_point)
        self.reference_power = -55.0
        """
        self.marker_size = 1

        # self.franunhofer_distance()
        self.is_dBm = True
        self.mode = True

        # Map variables
        self.map_origin = []
        self.map_resolution = 0
        self.map_size = []
        self.map_points = []
        self.explored_map = {}
        self.interpolated_map = {}
        self.explored_map_x_index = []
        self.explored_map_y_index = []
        self.explored_map_x_meters = []
        self.explored_map_y_meters = []
        self.marker_array = []
        self.publisher_dict = {}

        self.complete_map_power = {}

        self.gap_between_points = 1  # Will predict the value for 1 in 1 meters

        self.map_topic_name = '/map'
        self.radio_RSSI_signals_topic_name = '/radio/collected_data'
        self.got_interpolated_map = False

        self.predicted_map = []
        self.x_low = 0
        self.y_low = 0
        self.x_max = 0
        self.y_max = 0

        # Starting ROS
        self.radio_markers_publisher = None
        self.time_interval = 15
        rospy.init_node('radio_prediction_node', anonymous=True)
        rospy.loginfo("Starting radio prediction node")

        self.rate = rospy.Duration(self.time_interval, 0)
        self.rate_map = rospy.Duration(0.5, 0)
        self.map_timeout = 1

        # self.radio_markers_publisher = rospy.Publisher('/predicted_radio_markers',
        #                                                MarkerArray,
        #                                                queue_size=10,
        #                                                latch=True)

        self.rviz_makers_publisher = rospy.Publisher('/radio/predicted_map',
                                                     MarkerArray,
                                                     queue_size=10,
                                                     latch=True)

        self.complete_map_publisher = rospy.Publisher('/radio/complete_graph',
                                                      Float32MultiArray,
                                                      queue_size=10,
                                                      latch=True)

        rospy.loginfo("Service is ready to be called!")
        self.prediction_service = rospy.Service(
            "radio/get_predictions", Trigger, self.update_predicted_map)
        self.main_loop()

    def main_loop(self):

        while not rospy.is_shutdown():
            """
            Will call the function to update the radio map each 3 seconds
            """
            rospy.spin()
            # rospy.sleep(self.rate_map)

    def update_predicted_map(self, request):
        try:
    # First, we get the occupancy grid map
            self.initial_time = time.time()

            self.get_occupancy_map()

            # Next, we wait for the collecte radio signal power measurement
            markers = rospy.wait_for_message(self.radio_RSSI_signals_topic_name, MarkerArray, None)
            # Next, we will calculate the prediction for every suitable point

            # Will iterate over the explored map, from x to x meters
            for x in range(int(min(self.explored_map_x_meters)), int(max(self.explored_map_x_meters)), self.gap_between_points):
                for y in range(int(min(self.explored_map_y_meters)), int(max(self.explored_map_y_meters)), self.gap_between_points):

                    # Will get the index of the point to get later the key of the dictionary
                    index = radio_obj.map_to_coordinates(
                        x, y, self.map_origin, self.map_resolution)
                    key = radio_obj.map_idx(self.map_size[0], index[0], index[1])

                    # if this key (cell) is in the explored map, calculate the predicted value of the x meters cell
                    if key in self.explored_map:
                        distance = self.calculate_distance_between_antenna(
                            self.explored_map[key])
                        power = self.estimate_radio_signal(
                            self.explored_map[key], distance, markers)

                        # The resulting dictionary will be organized as: [x_meters, y_meters, power_dbm, index_i, index_j]
                        self.publisher_dict[key] = [self.explored_map[key][0], self.explored_map[key]
                                                    [1], power, self.explored_map[key][2], self.explored_map[key][3]]
            
            if self.debug:
                execution_time = time.time() - self.initial_time
                rospy.loginfo(
                    'Prediction process took %s seconds to be executed.', execution_time)
            self.get_whole_radio_map()
            rospy.loginfo("Published results")
            rospy.loginfo("Publishing predicted map")
            # self.pub_map_array()
            return([True, "Published results"])
            
        except Exception as error:
            rospy.logwarn("Could not get the predicted map")
            rospy.logerr(error)
            return([False, "Could not get the predicted map"])
            
    def get_interpolated_map(self):
        try:
            msg = rospy.wait_for_message(
                '/radio/interpolated_graph', Float32MultiArray, 1)
            for i in range(0, len(msg.data), 2):
                # self.interpolated_map[msg.data[i]] = msg.data[i+1]
                self.explored_map[msg.data[i]] = [self.explored_map[msg.data[i]][0], self.explored_map[msg.data[i]]
                                                    [1], msg.data[i+1], self.explored_map[msg.data[i]][2], self.explored_map[msg.data[i]][3]]
            # rospy.loginfo("Got interpolated map")
            self.got_interpolated_map = True

        except Exception as e:
            rospy.loginfo("Could not get the interpolated map...")

    def get_occupancy_map(self):
        try:
            msg = rospy.wait_for_message(
                self.map_topic_name, OccupancyGrid, None)
            self.map_origin = [msg.info.origin.position.x,
                               msg.info.origin.position.y]
            self.map_resolution = msg.info.resolution
            self.map_size = [msg.info.width, msg.info.height]
            self.map_points = msg.data

            self.explored_map = {}
            self.explored_map_x_index = []
            self.explored_map_y_index = []
            self.explored_map_x_meters = []
            self.explored_map_y_meters = []

            for i in range(msg.info.width):
                for j in range(msg.info.height):
                    if self.map_points[radio_obj.map_idx(self.map_size[0], i, j)] == 0:
                        x_aux, y_aux = radio_obj.coordinates_to_map(
                            i, j, self.map_origin, self.map_resolution)
                        self.explored_map_x_index.append(i)
                        self.explored_map_y_index.append(j)
                        self.explored_map_x_meters.append(x_aux)
                        self.explored_map_y_meters.append(y_aux)
                        self.explored_map[radio_obj.map_idx(self.map_size[0], i, j)] = [
                            x_aux, y_aux, i, j]

        except Exception as e:
            rospy.logerr("Map not avaliable!")
            print(e)

    def estimate_radio_signal(self, coordinates, distance, markers):
        closest_point = self.get_closest_point(coordinates, markers)
        predicted_power = self.log_shadow_equation(
            distance, closest_point[0], closest_point[1])
        # predicted_power = self.simple_log_shadow(distance)
        return predicted_power

    def friss_equation(self, d):
        self.PL_dB = (self.Gt_dBi +
                      self.Gr_dBi +
                      20 * math.log10(self.lmbda/(4*math.pi)) -
                      10 * self.n * math.log10(d) -
                      10 * math.log10(self.L))

        self.Pr_dBm = self.Pt_dbm + self.PL_dB

    def log_shadow_equation(self, d, closest_point, closest_point_power):
        closest_point_distance = self.calculate_distance_between_antenna(
            closest_point)
        path_loss = self.know_point_path_loss(
            closest_point_power) + (10 * self.n * math.log10(d/closest_point_distance))
        total_power = self.Pt_dbm + self.Gr_dBi + self.Gt_dBi - path_loss
        return total_power

    def simple_log_shadow(self, d):
        path_loss = self.know_point_path_loss(
            self.reference_power) + (10 * self.n * math.log10(d/(self.reference_point)))
        total_power = self.Pt_dbm + self.Gr_dBi + self.Gt_dBi - path_loss
        return total_power

    def get_closest_point(self, coordinates, markers):

        old_distance = math.inf
        closest_point = []
        power = []
        for i in range(len(markers.markers)):
            distance_calculated = self.calculate_distance_between_two_points(
                [markers.markers[i].pose.position.x, markers.markers[i].pose.position.y], coordinates)
            if distance_calculated < old_distance:
                power = (radio_obj.get_rssi_from_rgb(
                    markers.markers[i].color.r, self.is_dBm))
                closest_point = [markers.markers[i].pose.position.x,
                                 markers.markers[i].pose.position.y]
                old_distance = distance_calculated
        return [closest_point, power]

    def franunhofer_distance(self):
        self.df = (2*(self.largest_linear_antenna_dimension**2)) / self.lmbda

    def know_point_path_loss(self, received_power):
        pl = self.Pt_dbm + self.Gr_dBi + self.Gt_dBi - received_power
        # print("Know path loss:", pl)
        return pl

    def calculate_distance_between_two_points(self, position1, position2):
        return (math.sqrt((position1[0] - position2[0])**2 + (position1[1] - position2[1])**2))

    def calculate_distance_between_antenna(self, position):
        return (math.sqrt((self.base_antenna[0] - position[0])**2 + (self.base_antenna[1] - position[1])**2))

    def pub_map_array(self):
        radio_markers_array = MarkerArray()
        radio_markers_array.markers = []
        for index in self.publisher_dict:
            intensity = radio_obj.get_rgb_from_rssi(
                self.publisher_dict[index][2], self.is_dBm)
            radio_new_marker = Marker()
            radio_new_marker.header.stamp = rospy.Time.now()
            radio_new_marker.header.frame_id = "map"
            radio_new_marker.ns = "predicted_array"
            radio_new_marker.id = index
            radio_new_marker.type = Marker.CUBE
            radio_new_marker.action = Marker.MODIFY
            radio_new_marker.pose.position.x = self.publisher_dict[index][0]
            radio_new_marker.pose.position.y = self.publisher_dict[index][1]
            radio_new_marker.pose.position.z = 0
            radio_new_marker.pose.orientation.w = 1.0
            radio_new_marker.scale.x = self.marker_size
            radio_new_marker.scale.y = self.marker_size
            radio_new_marker.scale.z = 0.05
            radio_new_marker.color.r = 1.0 * intensity
            radio_new_marker.color.g = 0.8 * (1 - 2.0 * abs(intensity - 1 / 2))
            radio_new_marker.color.b = 1 - 1.0 * intensity
            radio_new_marker.color.a = 0.5
            radio_new_marker.lifetime = rospy.Duration(0)
            radio_markers_array.markers.append(radio_new_marker)
        self.rviz_makers_publisher.publish(radio_markers_array)


    def pub_complete_map_markers_debug(self):
        radio_markers_array = MarkerArray()
        radio_markers_array.markers = []
        self.get_map_limits()
        for i in range(self.x_low, self.x_max, 8):
            for j in range(self.y_low, self.y_max, 8):
                radio_new_marker = Marker()
                index = radio_obj.map_idx(self.map_size[0], i, j)
                if index in self.explored_map:
                    if len(self.explored_map[index]) > 4:
                        intensity = radio_obj.get_rgb_from_rssi(self.explored_map[index][2], self.is_dBm)
                        radio_new_marker = Marker()
                        radio_new_marker.header.stamp = rospy.Time.now()
                        radio_new_marker.header.frame_id = "map"
                        radio_new_marker.ns = "predicted_array"
                        radio_new_marker.id = index
                        radio_new_marker.type = Marker.CUBE
                        radio_new_marker.action = Marker.MODIFY
                        radio_new_marker.pose.position.x = self.explored_map[index][0]
                        radio_new_marker.pose.position.y = self.explored_map[index][1]
                        radio_new_marker.pose.position.z = 0
                        radio_new_marker.pose.orientation.w = 1.0
                        radio_new_marker.scale.x = 0.4
                        radio_new_marker.scale.y = 0.4
                        radio_new_marker.scale.z = 0.05
                        radio_new_marker.color.r = 1.0 * intensity
                        radio_new_marker.color.g = 0.8 * (1 - 2.0 * abs(intensity - 1 / 2))
                        radio_new_marker.color.b = 1 - 1.0 * intensity
                        radio_new_marker.color.a = 0.8
                        radio_new_marker.lifetime = rospy.Duration(0)
                        radio_markers_array.markers.append(radio_new_marker)
        self.rviz_makers_publisher.publish(radio_markers_array)

    def pub_complete_map(self):
        array_publish = []
        for index in self.explored_map:
            if len(self.explored_map[index]) > 4:
                array_publish.append(index)
                array_publish.append(self.explored_map[index][2])
        status = Float32MultiArray()
        status.layout.data_offset = 0
        status.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        status.layout.dim[0].label = "lines"
        status.layout.dim[0].size = 1
        status.layout.dim[0].stride = 3
        status.layout.dim[1].label = "columns"
        status.layout.dim[1].size = len(array_publish)
        status.layout.dim[1].stride = len(array_publish)
        status.data = array_publish
        self.complete_map_publisher.publish(status)
        # print("Tamanho da msg publicada: ", len(array_publish))
        # rospy.loginfo("Complet map published!")

    def get_map_limits(self):
        self.x_low = int(min(self.explored_map_x_index))
        self.y_low = int(min(self.explored_map_y_index))
        self.x_max = int(max(self.explored_map_x_index))
        self.y_max = int(max(self.explored_map_y_index))

    def get_whole_radio_map(self):
        scale = int((self.marker_size/2) / self.map_resolution) + 1
        for v in (self.publisher_dict):
            center_point = [self.publisher_dict[v]
                            [3], self.publisher_dict[v][4]]
            for i in range(center_point[0] - scale, center_point[0] + scale):
                for j in range(center_point[1] - scale, center_point[1] + scale):
                    index = radio_obj.map_idx(self.map_size[0], i, j)
                    if self.map_points[index] == 0:
                        power = self.publisher_dict[v][2]
                        self.explored_map[index] = [self.explored_map[index][0], self.explored_map[index]
                                                    [1], power, self.explored_map[index][2], self.explored_map[index][3]]
        self.get_interpolated_map()
        self.pub_complete_map()
        self.pub_complete_map_markers_debug()


if __name__ == '__main__':
    radio_obj = RadioLib()
    predictor = RadioPredictor()
