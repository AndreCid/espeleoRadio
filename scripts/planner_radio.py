#!/usr/bin/env python3
"""
Scrip to read the radio signals from interpolation markers in RVIZ and publish two paths: Shortest path between two
points and the optimum radio signal path

Can work in two modes: Robot mode and Static Mode;
    In robot mode, the initial point will be the robot pose and the final point can be choose in RVIZ.
"""

import rospy
import tf
import math

import networkx as nx

from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray, Bool
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point32, Pose
from espeleo_control.msg import Path as PathEspepelo
from radio_lib import RadioLib


class PathPlanner:
    def __init__(self):

        # Starting variables

        self.map_topic_name = '/map'
        self.radio_markers_topic_name = '/radio/complete_graph'
        self.is_dBm = True
        self.is_to_plot_shortest_path = False
        # Radio variables
        self.RSSI = {}
        self.first_time = 0

        self.listener = None

        # path variable

        #self.initial_point = [4.35, 1.2]
        #self.final_point = [-15.8, 2.3]
        self.initial_point = []
        self.final_point = []
        self.map_graph = nx.Graph()
        self.radio_graph = nx.Graph()
        self.starting_point_index = 0
        self.final_point_index = 0
        self.path_radio_signals = []
        self.power = 10
        self.robot_width = 1

        """ 3 possibles modes for the planner script:
                1 - robot:    
                        The starting point of the path will be the robot position on the map
                        and will publish the path topic every time a new goal is set.
                
                0 - Free:     
                        The starting point will be set in the map by rviz or by publishing in the topic 
                         /initialpose. The path will be published every time the initial point or the final point
                         is changed 

                2 - Navigation:
                        The initial point will be adressed by the navigation node and the goal will be ginven 
                        marking the map
        """
        self.planner_mode = 1

        # Map variables

        self.map_origin = []
        self.map_resolution = 0
        self.map_points = []
        self.map_size = []
        self.borders_to_be_removed = []

        self.radio_graph_is_ready = False
        self.map_graph_is_ready = False
        self.map_was_updated = False
        self.radio_path_is_ready = False

        # Starting ROS

        rospy.init_node('radio_path_planning_node', anonymous=True)
        rospy.loginfo("Planning node")

        # Initializing Subscribers

        self.subscriber_marker_array = rospy.Subscriber(self.radio_markers_topic_name,
                                                        Float32MultiArray,
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
        
        self.espeleo_shortest_path_publisher = rospy.Publisher('/espeleo_shortest_path',
                                                               PathEspepelo,
                                                               queue_size=10,
                                                               latch=True)

        self.espeleo_radio_path_publisher = rospy.Publisher('/espeleo_radio_path',
                                                            PathEspepelo,
                                                            queue_size=10,
                                                            latch=True)
        
        self.stop_navigation = rospy.Publisher('/stop_predictor', Bool, queue_size=10)

        self.time_interval = 1

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
        rospy.logwarn("Waiting map topic... ")
        while not self.map_graph_is_ready:
            rospy.sleep(self.rate)
        rospy.loginfo("Map topic received! Map graph initialized")

        # Waiting for interpolated marker
        rospy.logwarn("Waiting for interpolated radio map...")
        while not self.radio_graph_is_ready:
            rospy.sleep(self.rate)
        rospy.loginfo("Interpolated radio map received! Radio graph initialized")

        # Setting Start and goal points
        self.set_start_goal()

        while not rospy.is_shutdown():
            if self.planner_mode == 1:
                try:
                    #(trans,rot) = self.listener.lookupTransform('/map', 'base_link_2d', rospy.Time(0))
                    (trans,rot) = self.listener.lookupTransform('/map', 'path_origin', rospy.Time(0))
                    self.initial_point = [round(trans[0] / self.map_resolution) * self.map_resolution, round(trans[1] / self.map_resolution) * self.map_resolution]

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
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
            self.listener = tf.TransformListener()
            #rospy.Subscriber('/initialpose_planner', PoseWithCovarianceStamped, self.initial_pose_callback)
            #rospy.Subscriber("/tf", TFMessage, self.odom_callback)

        elif self.planner_mode == 2:
            rospy.loginfo("Navigation mode")
            rospy.Subscriber('/initialpose_planner', PoseWithCovarianceStamped, self.initial_pose_callback)

        else:
            rospy.logerr("Invalid planner mode")


        # final_point_msg = rospy.wait_for_message('/move_base_simple/goal', PoseStamped, 120)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.final_point_callback)

    def initial_pose_callback(self, msg):
        self.initial_point = [round(msg.pose.pose.position.x / self.map_resolution) * self.map_resolution,
                              round(msg.pose.pose.position.y / self.map_resolution) * self.map_resolution]
        if self.planner_mode == 2:
            self.radio_graph_is_ready = False
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
        if self.planner_mode == 2:
            self.radio_graph_is_ready = False
        self.starting_path_planning()

    def starting_path_planning(self):
        if self.planner_mode == 2:
            if self.first_time < 2:
                self.first_time = self.first_time + 1
            else:
                while not self.radio_graph_is_ready:
                    rospy.loginfo_once("Waiting for complete graph")
                rospy.loginfo("Received complete graph!")
        if self.final_point and self.initial_point:
            if self.is_to_plot_shortest_path:
                rospy.logwarn("Planning shortest path...")
                self.calculate_path(self.map_graph, self.shortest_path_publisher, self.espeleo_shortest_path_publisher)
            rospy.logwarn("Planning best signal path...")
            self.calculate_path(self.radio_graph, self.radio_path_publisher, self.espeleo_radio_path_publisher)
        elif not self.initial_point:
            rospy.logwarn("Waiting for the starting point \nSet in map with '2D Pose Estimate'"
                          " Rviz button or by publishing in the /initialpose topic: ")
        elif not self.final_point:
            rospy.logwarn("Waiting for the navigation goal \nSet in map with '2D Nav Goal'")

    def initialize_graph(self, graph, radio_weight=False):
        for i in self.RSSI:
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

        rospy.loginfo("Removing borders...")
        self.remove_graph_neighborhood(graph, self.robot_width, radio_weight)
        """
        Convert the graph to visualize in another progam - Cytoscape
        """
        #file_name = "Radio_graph.txt"
        #file = open(file_name, 'w')
        #file.write(str(nx.cytoscape_data(self.radio_graph)))
        #file.close()
        

    def calculate_path(self, graph, publisher, publisher_espeleo):
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
            self.plot_path(path, publisher, publisher_espeleo)

        except Exception as e:
            rospy.logerr("The starting and/or final point are not reachable. Please select other points")
            print(e)

    def plot_path(self, path, publisher, publisher_espeleo):
        ros_path = Path()
        ros_path.header.frame_id = 'map'
        ros_path.header.stamp = rospy.get_rostime()
        path_point = []

        espeleo_path = PathEspepelo()
        espeleo_path.header.stamp = rospy.get_rostime()
        espeleo_path.header.frame_id = 'map'
        espeleo_path.insert_n_points = 0
        espeleo_path.closed_path_flag = False

        for i in path:
            point = PoseStamped()
            point.header.frame_id = 'map'
            point_espeleo = Point32()

            y_aux = int(i/self.map_size[0])
            x_aux = i - (y_aux * self.map_size[0])
            x, y = radio_obj.coordinates_to_map(x_aux, y_aux, self.map_origin, self.map_resolution)
            point.pose.position.x = x
            point.pose.position.y = y
            path_point.append(point)

            point_espeleo.x = x
            point_espeleo.y = y
            point_espeleo.z = 0
            espeleo_path.path.points.append(point_espeleo)

        ros_path.poses = path_point
        publisher.publish(ros_path)
        publisher_espeleo.publish(espeleo_path)
        is_to_stop = Bool()
        is_to_stop.data = False
        self.stop_navigation.publish(is_to_stop)


    def subscriber_markers_callback(self, msg):
        # print("opa")
        #is_to_stop = Bool()
        #is_to_stop.data = True
        #self.stop_navigation.publish(is_to_stop)

        self.radio_graph = nx.Graph()
        for i in range(0, len(msg.data), 2):
            self.radio_graph.add_node(int(msg.data[i]))
            self.RSSI[msg.data[i]] = msg.data[i+1]
        #print(len(self.radio_graph.nodes))
        self.initialize_graph(self.radio_graph, True)
    
    def map_callback(self, msg):
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_resolution = msg.info.resolution
        self.map_size = [msg.info.width, msg.info.height]
        if self.is_to_plot_shortest_path:
            for i in range(msg.info.width):
                for j in range(msg.info.height):
                    map_index = radio_obj.map_idx(self.map_size[0], i, j)
                    if msg.data[map_index] == 0:
                        self.map_graph.add_node(map_index)
            self.initialize_graph(self.map_graph)
        else: 
            self.map_graph_is_ready = True
        self.map_was_updated = True
        

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

    def remove_graph_neighborhood(self, graph, robot_size, radio):
        if self.is_to_plot_shortest_path:
            layers = int(math.ceil(robot_size/(2*self.map_resolution)))
            for l in range(layers):
                to_be_removed = []
                for i in list(graph.nodes()):
                    if nx.degree(graph, i) < 8:
                        to_be_removed.append(i)
                graph.remove_nodes_from(to_be_removed)
            if radio:
                self.radio_graph_is_ready = True
            if not radio:
                self.map_graph_is_ready = True    

        else:
            if self.map_was_updated:
                layers = int(math.ceil(robot_size/(2*self.map_resolution)))
                for l in range(layers):
                    #self.borders_to_be_removed = []
                    for i in list(graph.nodes()):
                        if nx.degree(graph, i) < 8:
                            self.borders_to_be_removed.append(i)
                    graph.remove_nodes_from(self.borders_to_be_removed)
                self.map_was_updated = False
            else: 
                graph.remove_nodes_from(self.borders_to_be_removed)
            if radio:
                self.radio_graph_is_ready = True
            if not radio:
                self.map_graph_is_ready = True
        print("Tamanho do grafo", len(graph))
        


if __name__ == '__main__':
    radio_obj = RadioLib()
    map_obj = PathPlanner()
