#!/usr/bin/env python3

"""
Programa para duplicar árvore de TF para utilizar Gmappin 

"""

import rospy
import tf


from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from radio_lib import RadioLib


class TfDuplicator:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.current_time = 0

        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.init_node('transform_3d_to_2d', anonymous=True)
        rospy.loginfo("Starting duplicate TF trees!")

        self.subscriber_odom = rospy.Subscriber("/ekf_loam/integrated_to_init",
                                                Odometry,
                                                self.odom_callback)
        while not rospy.is_shutdown():
            rospy.spin()

    def odom_callback(self, msg):
        self.odom_broadcaster.sendTransform(
            (msg.pose.pose.position.x, msg.pose.pose.position.y,
             msg.pose.pose.position.z),
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            rospy.Time.now(),
            "base_link_2d",
            "odom_2d"
        )


if __name__ == '__main__':
    predictor = TfDuplicator()
    radio_obj = RadioLib()
