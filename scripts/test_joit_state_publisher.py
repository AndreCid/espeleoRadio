#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState


def talker():
    pub = rospy.Publisher('device1/get_joint_state', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)  # 10hz
    seq = 0
    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.header.seq = seq
        msg.velocity = [seq]
        pub.publish(msg)
        seq = seq + 1
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
