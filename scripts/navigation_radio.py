#!/usr/bin/env python3
"""
Program to manage all services:

    - call interpolation service;
    - call predction service;
    - call planner given a goal point
"""

import rospy
import math
import tf

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Trigger



class RadioNavigation:
    def __init__(self):
        
        self.interpolation_service = "radio/get_interpolation"
        self.predictor_service = "radio/get_predictions" 
        #self.planner_service = "radio/plan_path"
        self.map_resolution = 0.05

        self.distance_traveled = 0
        self.ditance_threshold = 3

        self.first_time = True
        self.paused = False

        rospy.init_node('radio_navigation_node', anonymous=True)
        
        
        rospy.wait_for_service(self.predictor_service)
        rospy.wait_for_service(self.interpolation_service)

        #rospy.Subscriber("/tf", TFMessage, self.odom_callback)
        
        

        self.predict_power = rospy.ServiceProxy(self.predictor_service, Trigger)
        self.interpolated_power = rospy.ServiceProxy(self.interpolation_service, Trigger)
        self.initial_point = [0, 0]
        self.tf_point = [0,0]
        self.last_point = [0,0]


        self.new_position = rospy.Publisher('/initialpose_planner',PoseWithCovarianceStamped, queue_size=10)
        self.stop_movement = rospy.Publisher('/espeleo/vecfield_enable', Bool, queue_size=10)
        self.amcl_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.get_amcl_pose)
        self.get_status  = rospy.Subscriber('/stop_predictor',Bool, self.change_status)

        self.main_loop()
        #try:
        #    x = was_predicted()
        #    print(x)
        #except:
        #    print(was_predicted)

    def main_loop(self):
        rate = rospy.Duration(0.1)
        self.listener = tf.TransformListener()
        rospy.loginfo("Starting radio navigation system!")
        while not rospy.is_shutdown():
            if not self.paused:
                try:
                    (trans,rot) = self.listener.lookupTransform('/map', 'path_origin', rospy.Time(0))
                    self.tf_point = [round(trans[0] / self.map_resolution) * self.map_resolution, round(trans[1] / self.map_resolution) * self.map_resolution]
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            rospy.sleep(rate)

    def get_amcl_pose(self,msg):
        self.initial_point = [round(msg.pose.pose.position.x / self.map_resolution) * self.map_resolution, round(msg.pose.pose.position.y  / self.map_resolution) * self.map_resolution]
        if self.first_time:
                        print("first time")
                        actual_position = PoseWithCovarianceStamped()
                        actual_position.header.stamp = rospy.Time.now()
                        actual_position.header.frame_id = '/map'
                        actual_position.pose.pose.position.x = self.initial_point[0]
                        actual_position.pose.pose.position.y = self.initial_point[1]
                        self.new_position.publish(actual_position)
                        self.last_point = list(self.initial_point)
                        self.first_time = False
                        rospy.loginfo("Getting robot position")
        td = math.sqrt((round(self.initial_point[0],1) - round(self.last_point[0],1))**2 + (round(self.initial_point[1],1) - round(self.last_point[1],1))**2)
        self.distance_traveled = self.distance_traveled + td 
        print(self.distance_traveled)
        self.last_point = list(self.initial_point)
        if self.distance_traveled > self.ditance_threshold:
            self.update_path()
            self.distance_traveled = 0

    # def odom_callback(self, msg):
    #     for T in msg.transforms:
    #         # Choose the transform of the robot
    #         if T.child_frame_id == "base_link_2d":
    #             # Get the position
    #             self.initial_point = [round(T.transform.translation.x / self.map_resolution) * self.map_resolution,
    #                                   round(T.transform.translation.y / self.map_resolution) * self.map_resolution]
                
    def update_path(self):
        self.paused = True
        enable_controll = Bool()
        enable_controll.data = False
        self.stop_movement.publish(enable_controll)
        rospy.loginfo("Updating maps")
        try:
            """
            Get position first, will be used as reference before keep walking and collecting data
            """
            actual_position = PoseWithCovarianceStamped()
            actual_position.header.stamp = rospy.Time.now()
            actual_position.header.frame_id = '/map'
            actual_position.pose.pose.position.x = self.tf_point[0]
            actual_position.pose.pose.position.y = self.tf_point[1]
            was_interpolated = self.interpolated_power()
            print(was_interpolated.message)
            if was_interpolated.success:
                was_predicted = self.predict_power()
                if was_predicted.success:
                    print(was_predicted.message)
                    
                    self.new_position.publish(actual_position)
                    rospy.loginfo("New position seted")
        except Exception as error:
            rospy.logwarn(error)
        
    def change_status(self,msg):
        self.paused = msg.data

if __name__ == '__main__':
    map_obj = RadioNavigation()
