#!/usr/bin/env python3

"""
Programa para duplicar árvore de TF para utilizar Gmappin 

"""

import rospy
import tf


from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from radio_lib import RadioLib


class TfDuplicator:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.current_time = 0

        rospy.init_node('remap_scan', anonymous=True)

        self.subscriber_scan = rospy.Subscriber("/scan",
                                                LaserScan,
                                                self.scan_callback)
        self.new_scan_publisher = rospy.Publisher("/scan_2d", 
                                                  LaserScan,
                                                  queue_size=10)
        
        while not rospy.is_shutdown():
            rospy.spin()

    def scan_callback(self, msg):
        scan = LaserScan()
        scan = msg
        scan.header.frame_id = "velodyne_2d"
        scan.header.stamp = rospy.Time.now()
        self.new_scan_publisher.publish(scan)


if __name__ == '__main__':
    predictor = TfDuplicator()
    radio_obj = RadioLib()
