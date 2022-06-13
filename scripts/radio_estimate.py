#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from tf2_msgs.msg import TFMessage
from math import log10, sqrt

d = 0.0

def callback_odom(data):
    
    for T in data.transforms:
        # Choose the transform of the EspeleoRobo
        if (T.child_frame_id == "base_link"):
            # Get the position
            x = T.transform.translation.x
            y = T.transform.translation.y

            global d
            # Calculating the distance
            obs = rospy.get_param("obstacle")
            if obs:
                d = abs(x-xb) + abs(y-yb)
            else:
                #d = abs(sqrt(x**2 + y**2) - db)
                d = sqrt((x-xb)**2 +(y-yb)**2)

    return

def estimate():
    # Criando a variavel que contem os status da conexao que serao publicados
    status = Float32MultiArray()
    status.layout.data_offset = 0
    status.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
    # dim[0] is the vertical dimension of your matrix
    status.layout.dim[0].label = "lines"
    status.layout.dim[0].size = 1
    status.layout.dim[0].stride = 3
    # dim[1] is the horizontal dimension of your matrix
    status.layout.dim[1].label = "columns"
    status.layout.dim[1].size = 3
    status.layout.dim[1].stride = 3
    status.data = [0.0, 70.0, 0.0]
    
    rospy.init_node('estimate_RSS', anonymous=True)

    pub = rospy.Publisher('/radio/status', Float32MultiArray, queue_size=10)

    odom_sub = rospy.Subscriber(odom_topic, TFMessage, callback_odom)

    node_sleep_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if linear_model:
            RSS = 80 - 1.41*d 
        else:
            if d <= 1:
                RSS = 100
            else:
                RSS = 100 - 2.1*10*log10(d)
        
        status.data[0] = RSS
        pub.publish(status)
        node_sleep_rate.sleep()


if __name__ == '__main__':

    xb = float(rospy.get_param("xb"))
    yb = float(rospy.get_param("yb"))
    linear_model = rospy.get_param("linear_model")
    odom_topic = rospy.get_param("odom_topic")

    global db
    # Calculating the base distance
    db = (xb**2 + yb**2)**(1/2)

    try:
        estimate()
    except rospy.ROSInterruptException:
        pass