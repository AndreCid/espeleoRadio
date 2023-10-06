#!/usr/bin/env python3
"""
Scrip to read the radio signals from markers in RVIZ and data from the map and create a interpolation map

"""

import rospy
import time

import matplotlib.pyplot as plt
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
from scipy.interpolate import griddata
from radio_lib import RadioLib


class MapInterpolation:
    def __init__(self):

        # Starting variables

        self.map_topic_name = '/map'
        self.radio_RSSI_signals_topic_name = '/radio/collected_data'
        self.is_dBm = True

        # Radio variables
        self.radio_RSSI_signals = []
        self.x = []
        self.y = []
        self.RSSI = []
        self.radio_map = []
        self.first_time = True
        self.radio_graph_array = []

        # Heatmap variables
        self.interpolation_step = 8
        self.marker_size = 0.4
        self.x_low = 0
        self.y_low = 0
        self.x_max = 0
        self.y_max = 0

        # Map variables
        self.map_origin = []
        self.map_resolution = 0
        self.map_points = []
        self.map_size = []
        self.explored_map = []
        self.explored_map_x = []
        self.explored_map_y = []

        # debug_variables
        self.debug = True
        self.initial_time = None

        # Starting ROS
        self.time_interval = 1
        self.maps_acquisition_time_interval = 0.5
        rospy.init_node('radio_interpolation_node', anonymous=True)
        rospy.loginfo("Starting radio interpolation mapping node")

        self.rate = rospy.Duration(self.time_interval, 0)
        self.acquisitioin_rate = rospy.Duration(self.maps_acquisition_time_interval, 0)

        """
        Creating publishers:
        Due to Rviz performance, we need to create one marker array publishers:

            - Visualization Radio Markers: Used for visualization purpose only, having inflated markers and 
                increasing the gap between every marker to decrease the number of markers. 
                Rviz tends to freeze and slow down when a marker array with a lot of points is .

            - Interpolated graph: The main array, containing all the data from interpolation
                having an array with both measure and correspondent occupancy grid ID.
                                   
        """
        self.radio_markers_publisher = rospy.Publisher('/radio/interpolated_graph',
                                                       Float32MultiArray,
                                                       queue_size=10,
                                                       latch=True)

        self.rviz_makers_publisher = rospy.Publisher('/radio/interpolated_map',
                                                     MarkerArray,
                                                     queue_size=10,
                                                     latch=True)

        # Initializing Map subscriber first, and then the radio subscriber (radio sub  callback needs maps information)

        #self.subscriber_map = rospy.Subscriber(self.map_topic_name,
        #                                       OccupancyGrid,
        #                                       self.map_callback)
        
        self.prediction_service = rospy.Service("radio/get_interpolation", Trigger, self.update_interpolation_map)
        rospy.loginfo("Interpolation service is ready to be called!")

       # while not self.map_points:
        #    rospy.logwarn("Waiting map topic... ")
        #    rospy.sleep(self.acquisitioin_rate)

        #rospy.loginfo("Received map topic! Waiting for radio markers topic...")
        #self.subscriber_radio = rospy.Subscriber(self.radio_RSSI_signals_topic_name,
        #                                         MarkerArray,
        #                                         self.subscriber_radio_callback)
        # Waiting for radio signal marker
        #while not self.RSSI:
        #    rospy.logwarn("Waiting for radio markers... ")
        #    rospy.sleep(self.acquisitioin_rate)
        #rospy.loginfo("Interpolation process completed!")
        self.init_ros()

    # Initializing main loop
    def init_ros(self):
        while not rospy.is_shutdown():
            rospy.spin()


    def update_interpolation_map(self, request):
        try:
            self.get_occupancy_map()
            self.get_radio_collected_data()
            return([True, "Interpolation process completed"])
        except Exception as error:
            return([False, "error"])
            

    """
    Map callback function:
        - Receive the map message with the map information.
        - Use a function from radio lib to transform the x, y matrix in a single index array
            (Occupancy Grid uses a single dimension array for the map points)
        - Transform the i and j index in X and Y (meters) coordinates 
        - Get only the explored map to optimize the interpolation process 
    """

    def get_occupancy_map(self):
        msg = msg = rospy.wait_for_message(self.map_topic_name, OccupancyGrid, None)
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_resolution = msg.info.resolution
        self.map_size = [msg.info.width, msg.info.height]
        self.map_points = msg.data

        for i in range(msg.info.width):
            for j in range(msg.info.height):
                if self.map_points[radio_obj.map_idx(self.map_size[0], i, j)] == 0:
                    x_aux, y_aux = radio_obj.coordinates_to_map(i, j, self.map_origin, self.map_resolution)
                    self.explored_map_x.append(x_aux)
                    self.explored_map_y.append(y_aux)
        self.explored_map = [self.explored_map_x, self.explored_map_y]
        """
        DEBUG USE: 
            - Plot to check if the coordinates matches the explored map  
        
        plt.plot(self.explored_map_x, self.explored_map_y, 'r.')
        plt.axis([self.map_origin[0],
                  self.map_origin[0] + self.map_size[0] * self.map_resolution,
                  self.map_origin[1],
                  self.map_origin[1] + self.map_size[1] * self.map_resolution])
        plt.show()
        """

    """
    Radio marker callback:
        - Receive the marker array
        - Catches all the marker from the marker array for the first time and then append every new marker
        - Get the Red RGB value (0 - 1) and remap (0 - 100) 
    """

    def get_radio_collected_data(self):
        msg = rospy.wait_for_message(self.radio_RSSI_signals_topic_name, MarkerArray, None)
        if self.first_time:
            rospy.loginfo("Received radio markers!")
            rospy.loginfo("Starting interpolation process:")
            self.x = []
            self.y = []
            self.radio_map = []
            self.RSSI = []
            for i in range(len(msg.markers)):
                x, y = radio_obj.map_to_coordinates(msg.markers[i].pose.position.x,
                                                    msg.markers[i].pose.position.y,
                                                    self.map_origin,
                                                    self.map_resolution)
                self.x.append(x)
                self.y.append(y)
                self.radio_map.append([self.x, self.y])
                self.RSSI.append(radio_obj.get_rssi_from_rgb(msg.markers[i].color.r, self.is_dBm))

            #self.first_time = False
        else:
            x, y = radio_obj.map_to_coordinates(msg.markers[-1].pose.position.x,
                                                msg.markers[-1].pose.position.y,
                                                self.map_origin,
                                                self.map_resolution)

            self.x.append(x)
            self.y.append(y)
            self.radio_map.append([self.x, self.y])
            self.RSSI.append(radio_obj.get_rssi_from_rgb(msg.markers[-1].color.r, self.is_dBm))

        if len(self.RSSI) < 4:
            rospy.logwarn("Waiting for more thant 4 radio signal points...")
        else:
            self.linear_interpolation()

    """
    Linear interpolation
        - Get the max and min points from the array in both directions (x and y) to minimize the interpolation
        - Create a grid with this values
        - Apply a simple interpolation process (3 available from this method: Linear, Cubic and Nearest Neighbor
        - Publish first the complete array and then the visualization array     
    """

    def linear_interpolation(self):

        self.initial_time = time.time()

        self.x_low = int(min(self.x))
        self.y_low = int(min(self.y))
        self.x_max = int(max(self.x))
        self.y_max = int(max(self.y))

        grid_x, grid_y = np.mgrid[self.x_low:self.x_max, self.y_low:self.y_max]
        z = griddata((self.x, self.y), self.RSSI, (grid_x, grid_y), method='linear')
        self.pub_array(z, self.interpolation_step, self.interpolation_step, self.marker_size)
        self.pub_interpolated_graph(z)

        if self.debug:
            execution_time = time.time() - self.initial_time
            rospy.loginfo(
                'Interpolation and publisher process took %s seconds to be executed. \nMap size: %s \nNumber of radio data: %s',
                execution_time, len(self.map_points), len(self.RSSI))

    """
    Array Publisher:
        - Initialize a marker and a marker array Ros msg
        - Populate the marker array with the interpolated points
        - RGB values are calculated so that the radio signal quality will vary the color 
            from red to blue (good to bad signal)          
    """

    def pub_array(self, z, step_x, step_y, marker_size):
        radio_markers_array = MarkerArray()
        radio_markers_array.markers = []
        for i in range(self.x_low, self.x_max, step_x):
            for j in range(self.y_low, self.y_max, step_y):
                index = radio_obj.map_idx(self.map_size[0], i, j)
                if self.map_points[index] == 0:
                    x, y = radio_obj.coordinates_to_map(i, j, self.map_origin, self.map_resolution)
                    intensity = radio_obj.get_rgb_from_rssi(z[i - self.x_low][j - self.y_low], self.is_dBm)
                    if (not np.isnan(intensity)) and (not np.isinf(intensity)):
                        # radio_new_marker = Marker()
                        if intensity > 1:
                            intensity = 1
                        elif intensity < 0:
                            intensity = 0
                        radio_new_marker = Marker()
                        radio_new_marker.header.stamp = rospy.Time.now()
                        radio_new_marker.header.frame_id = "map"
                        radio_new_marker.ns = "Interpolated"
                        radio_new_marker.id = index
                        radio_new_marker.type = Marker.CUBE
                        radio_new_marker.action = Marker.MODIFY
                        radio_new_marker.pose.position.x = x
                        radio_new_marker.pose.position.y = y
                        radio_new_marker.pose.position.z = 0
                        radio_new_marker.pose.orientation.w = 1.0
                        radio_new_marker.scale.x = marker_size
                        radio_new_marker.scale.y = marker_size
                        radio_new_marker.scale.z = 0.05
                        radio_new_marker.color.r = 1.0 * intensity
                        radio_new_marker.color.g = 0.8 * (1 - 2.0 * abs(intensity - 1 / 2))
                        radio_new_marker.color.b = 1 - 1.0 * intensity
                        radio_new_marker.color.a = 0.8
                        radio_new_marker.lifetime = rospy.Duration(0)
                        radio_markers_array.markers.append(radio_new_marker)
        self.rviz_makers_publisher.publish(radio_markers_array)

    '''
    Array Publisher:
        - Populate an array with [id_0, power_0, id_1, power_1, ..., id_n, power_n]
        - Only points that do not correspond to an obstacle
        - Publish an array with data -> Marker array has to many unecessary fields, making 
            the message to slow to be transported.        
    '''

    def pub_interpolated_graph(self, z):
        array_publish = []
        min_power, max_power = radio_obj.get_limits()
        for i in range(self.x_low, self.x_max):
            for j in range(self.y_low, self.y_max):
                index = radio_obj.map_idx(self.map_size[0], i, j)
                if self.map_points[index] == 0:
                    intensity = z[i - self.x_low][j - self.y_low]
                    if (not np.isnan(intensity)) and (not np.isinf(intensity)):
                        if intensity > max_power:
                            intensity = max_power
                        elif intensity < min_power:
                            intensity = min_power
                        array_publish.append(index)
                        array_publish.append(intensity)
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
        self.radio_markers_publisher.publish(status)


"""
Initialize the radio and map objects
"""
if __name__ == '__main__':
    radio_obj = RadioLib()
    map_obj = MapInterpolation()
