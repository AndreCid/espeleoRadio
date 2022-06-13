#!/usr/bin/env python
"""
Scrip to read the radio signals from interpolation markers in RVIZ and publish two paths: Shortest path between two
points and the optimum radio signal path

Can work in two modes: Robot mode and Static Mode;
    In robot mode, the initial point will be the robot pose and the final point can be choose in RVIZ.
"""

import rospy
import math

import networkx as nx

from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from radio_lib import RadioLib


class PathPlanner:
    def __init__(self):

        # Starting variables

        self.map_topic_name = '/map'
        self.radio_markers_topic_name = '/interpolated_radio_markers'

        # Radio variables
        self.x = []
        self.y = []
        self.RSSI = {}
        self.radio_map = []
        self.first_time = True

        self.x_size = 0
        self.y_size = 0

        self.x_low = 0
        self.y_low = 0
        self.x_max = 0
        self.y_max = 0

        # path variable

        # self.initial_point = [2.5, -4]
        # self.final_point = [-14.1, -8.0]
        self.initial_point = []
        self.final_point = []
        self.map_graph = nx.Graph()
        self.radio_graph = nx.Graph()
        self.starting_point_index = 0
        self.final_point_index = 0
        self.path_radio_signals = []
        self.power = 10
        self.robot_width = 1

        """ Two possibles modes for the planner script:
                1 - robot:    The starting point of the path will be the robot position on the map
                         and will publish the path topic every time a new goal is set.
                
                0 - Free:     The starting point will be set in the map by rviz or by publishing in the topic 
                         /initialpose. The path will be published every time the initial point or the final point
                         is changed 
        """
        self.planner_mode = 0

        # Map variables

        self.map_origin = []
        self.map_resolution = 0
        self.map_points = []
        self.map_size = []
        self.explored_map = []
        self.explored_map_x = []
        self.explored_map_y = []
        self.map_index = []
        self.radio_data_size = 0

        # Starting ROS

        rospy.init_node('radio_path_planning_node', anonymous=True)
        rospy.loginfo("Planning node")

        # Initializing Subscribers

        self.subscriber_marker_array = rospy.Subscriber(self.radio_markers_topic_name,
                                                        MarkerArray,
                                                        self.subscriber_markers_callback)

        self.subscriber_map = rospy.Subscriber(self.map_topic_name,
                                               OccupancyGrid,
                                               self.map_callback)

        # Initializing Publishers

        self.shortest_path_publisher = rospy.Publisher('/shortest_path',
                                                       Path,
                                                       queue_size=10,
                                                       latch=True)

        self.radio_path_publisher = rospy.Publisher('/radio_path',
                                                    Path,
                                                    queue_size=10,
                                                    latch=True)

        self.time_interval = 5

        self.rate = rospy.Duration(self.time_interval, 0)
        self.ros_radio_node()

    """
    Ros radio node:
        Function responsible to manage the ros node.
            - Will wait the map topic and marker array with interpolated data to start the planner
            - 
            
    
    """
    def ros_radio_node(self):

        # Waiting for map topic
        while not self.map_points:
            rospy.logwarn("Waiting map topic... ")
            rospy.sleep(self.rate)
        rospy.loginfo("Map topic received! Map graph initialized")

        # Waiting for interpolated marker
        while not self.radio_graph:
            rospy.logwarn("Waiting for interpolated radio map...")
            rospy.sleep(self.rate)
        rospy.loginfo("Interpolated radio map received! Radio graph initialized")

        # Setting Start and goal points
        self.set_start_goal()

        while not rospy.is_shutdown():
            rospy.sleep(self.rate)

    def set_start_goal(self):
        if self.planner_mode == 0:
            # Getting initial point
            rospy.logwarn("Waiting for the starting point \nSet in map with '2D Pose Estimate'"
                          " Rviz button or by publishing in the /initialpose topic: ")
            rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_pose_callback)
            # initial_point_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped, 120)

        elif self.planner_mode == 1:
            rospy.logwarn("Getting robot pose...")
            rospy.Subscriber("/tf", TFMessage, self.odom_callback)

        else:
            rospy.logerr("Invalid planner mode")

        # final_point_msg = rospy.wait_for_message('/move_base_simple/goal', PoseStamped, 120)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.final_point_callback)

    def initial_pose_callback(self, msg):
        self.initial_point = [round(msg.pose.pose.position.x / self.map_resolution) * self.map_resolution,
                              round(msg.pose.pose.position.y / self.map_resolution) * self.map_resolution]

        self.starting_path_planning()

    def odom_callback(self, msg):
        for T in msg.transforms:
            # Choose the transform of the robot
            if T.child_frame_id == "base_link":
                # Get the position
                self.initial_point = [round(T.transform.translation.x / self.map_resolution) * self.map_resolution,
                                      round(T.transform.translation.y / self.map_resolution) * self.map_resolution]

    def final_point_callback(self, msg):
        self.final_point = [round(msg.pose.position.x / self.map_resolution) * self.map_resolution,
                            round(msg.pose.position.y / self.map_resolution) * self.map_resolution]
        self.starting_path_planning()

    def starting_path_planning(self):
        if self.final_point and self.initial_point:
            rospy.logwarn("Planning shortest path...")
            self.calculate_path(self.map_graph, self.shortest_path_publisher)
            rospy.logwarn("Planning best signal path...")
            self.calculate_path(self.radio_graph, self.radio_path_publisher)
        elif not self.initial_point:
            rospy.logwarn("Waiting for the starting point \nSet in map with '2D Pose Estimate'"
                          " Rviz button or by publishing in the /initialpose topic: ")
        elif not self.final_point:
            rospy.logwarn("Waiting for the navigation goal \nSet in map with '2D Nav Goal'")

    def initialize_graph(self, graph, radio_weight=False):
        y_min = min(self.explored_map_y)
        y_max = max(self.explored_map_y)
        x_min = min(self.explored_map_x)
        x_max = max(self.explored_map_x)
        for y in range(y_min, y_max):
            for x in range(x_min, x_max):
                i = radio_obj.map_idx(self.map_size[0], x, y)
                if graph.has_node(i):

                    # Right neighbor
                    if graph.has_node(i + 1):
                        graph.add_edge(i, i + 1,
                                       weight=((100 - self.RSSI[(i + 1)])**self.power) if radio_weight
                                       else self.map_resolution)
                    # Left neighbor
                    if graph.has_node(i - 1):
                        graph.add_edge(i, i - 1,
                                       weight=((100 - self.RSSI[(i - 1)])**self.power) if radio_weight
                                       else self.map_resolution)
                    # Up neighbor
                    if graph.has_node(i + self.map_size[0]):
                        graph.add_edge(i, i + self.map_size[0],
                                       weight=((100 - self.RSSI[(i + self.map_size[0])])**self.power) if radio_weight
                                       else self.map_resolution)
                    # Down neighbor
                    if graph.has_node(i - self.map_size[0]):
                        graph.add_edge(i, i - self.map_size[0],
                                       weight=((100 - self.RSSI[(i - self.map_size[0])])**self.power) if radio_weight
                                       else self.map_resolution)

                    # Up-right neighbor
                    if graph.has_node(i + self.map_size[0] + 1):
                        graph.add_edge(i, i + self.map_size[0] + 1,
                                       weight=(((100 - self.RSSI[(i + self.map_size[0]) + 1]) * math.sqrt(2))
                                               ** self.power) if radio_weight else (math.sqrt(2) * self.map_resolution))

                    # Up-left neighbor
                    if graph.has_node(i + self.map_size[0] - 1):
                        graph.add_edge(i, i + self.map_size[0] - 1,
                                       weight=(((100 - self.RSSI[(i + self.map_size[0]) - 1]) * math.sqrt(2))
                                               ** self.power) if radio_weight else (math.sqrt(2) * self.map_resolution))

                    # Down-right neighbor
                    if graph.has_node(i - self.map_size[0] + 1):
                        graph.add_edge(i, i - self.map_size[0] + 1,
                                       weight=(((100 - self.RSSI[(i - self.map_size[0]) + 1]) * math.sqrt(2))
                                               ** self.power) if radio_weight else (math.sqrt(2) * self.map_resolution))

                    # Down-left neighbor
                    if graph.has_node(i - self.map_size[0] - 1):
                        graph.add_edge(i, i - self.map_size[0] - 1,
                                       weight=(((100 - self.RSSI[(i - self.map_size[0]) - 1]) * math.sqrt(2))
                                               ** self.power) if radio_weight else (math.sqrt(2) * self.map_resolution))

        self.remove_graph_neighborhood(graph, self.robot_width)

    def calculate_path(self, graph, publisher):
        rospy.loginfo("Starting point: " + str(self.initial_point))
        rospy.loginfo("Goal: " + str(self.final_point))

        self.starting_point_index = self.get_goal_points(self.initial_point)
        self.final_point_index = self.get_goal_points(self.final_point)
        self.path_radio_signals = []
        try:
            path = nx.dijkstra_path(graph, self.starting_point_index, self.final_point_index)
            total_distance = 0
            prev_node = 0
            for i in path:
                self.path_radio_signals.append(self.RSSI[i])
                dif = i - prev_node
                if self.is_side_neighbor(dif):
                    total_distance = total_distance + self.map_resolution
                elif self.is_diagonal_neighbor(dif):
                    total_distance = total_distance + (self.map_resolution * math.sqrt(2))
                prev_node = i

            rospy.loginfo("Path planned!")
            rospy.loginfo("\n PATH DATA: \n Path length: %s \n Total distance(meters): %s", len(path), total_distance)
            rospy.loginfo("\n RADIO DATA: \n "
                          "Maximum radio signal: %s \n "
                          "Minimum radio signal: %s \n "
                          "Total radio signal: %s \n "
                          "Average radio signal: %s\n\n",
                          max(self.path_radio_signals),
                          min(self.path_radio_signals),
                          sum(self.path_radio_signals),
                          (sum(self.path_radio_signals)/len(self.path_radio_signals)))
            self.plot_path(path, publisher)

        except Exception as e:
            rospy.logerr("The starting and/or final point are not reachable. Please select other points")
            print(e)

    def plot_path(self, path, publisher):
        ros_path = Path()
        ros_path.header.frame_id = 'map'
        ros_path.header.stamp = rospy.get_rostime()
        path_point = []
        for i in path:
            point = PoseStamped()
            point.header.frame_id = 'map'
            y_aux = int(i/self.map_size[0])
            x_aux = i - (y_aux * self.map_size[0])
            x, y = radio_obj.coordinates_to_map(x_aux, y_aux, self.map_origin, self.map_resolution)
            point.pose.position.x = x
            point.pose.position.y = y
            path_point.append(point)
        ros_path.poses = path_point
        publisher.publish(ros_path)

    def map_callback(self, msg):
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_resolution = msg.info.resolution
        self.map_size = [msg.info.width, msg.info.height]
        self.map_points = msg.data
        for i in range(msg.info.width):
            for j in range(msg.info.height):
                map_index = radio_obj.map_idx(self.map_size[0], i, j)
                if self.map_points[map_index] == 0:
                    self.map_graph.add_node(map_index)
                    self.explored_map_x.append(i)
                    self.explored_map_y.append(j)
        self.explored_map = [self.explored_map_x, self.explored_map_y]
        self.initialize_graph(self.map_graph)

    def subscriber_markers_callback(self, msg):
        for i in range(len(msg.markers)):
            self.radio_graph.add_node(msg.markers[i].id)
            self.RSSI[msg.markers[i].id] = (radio_obj.get_rssi_from_rgb(msg.markers[i].color.r))
        self.initialize_graph(self.radio_graph, True)

    def get_goal_points(self, point):
        # get i and j matrix index from x and y coordinate plan
        i, j = radio_obj.map_to_coordinates(point[0], point[1], self.map_origin, self.map_resolution)
        # get the graph id relative to i and j index
        point = radio_obj.map_idx(self.map_size[0], i, j)
        return point

    def is_side_neighbor(self, dif):
        if (abs(dif) == 1) or (abs(dif) == self.map_size[0]):
            return True
        else:
            return False

    def is_diagonal_neighbor(self, dif):
        if (dif == self.map_size[0] + 1) or (dif == self.map_size[0] - 1):
            return True
        elif (dif == -self.map_size[0] + 1) or (dif == -self.map_size[0] - 1):
            return True
        else:
            return False

    def remove_graph_neighborhood(self, graph, robot_size):
        layers = int(math.ceil(robot_size/(2*self.map_resolution)))
        for l in range(layers):
            to_be_removed = []
            for i in list(graph.nodes()):
                if nx.degree(graph, i) < 8:
                    to_be_removed.append(i)
            graph.remove_nodes_from(to_be_removed)


if __name__ == '__main__':
    radio_obj = RadioLib()
    map_obj = PathPlanner()
