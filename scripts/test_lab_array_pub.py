#!/usr/bin/env python
"""
Scrip to read the radio signals and plot then in a array of markers in RVIZ

"""

import rospy

import sys, select, termios, tty, math, message_filters

from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray


class RadioSignal:
    def __init__(self):

        # Starting variables

        # Class variables
        self.radio_RSSI = 0  # Instant radio signal intensity
        self.x = 0  # Instant robot position (x)
        self.y = 0
        self.radio_max = 100
        self.radio_min = 84
        self.green_max = 1
        self.green_min = 0
        self.OldRange = self.radio_max - self.radio_min
        self.NewRange = self.green_max - self.green_min
        self.first_time = True

        self.points = [[0, 0]]  # Array containing robot positions
        self.RSSI = [98.0,97.0,98.0,99.0,94.0,99.0,99.0,96.0,97.0,99.0,98.0,97.0,91.0,96.0,97.0,98.0,98.0,98.0,98.0,
                     88.0,97.0,97.0,96.0,99.0,86.0,96.0,99.0,92.0,98.0,99.0,95.0,93.0,99.0,98.0,99.0,99.0,99.0,91.0,
                     98.0,92.0,95.0,99.0,98.0,99.0,99.0,94.0,90.0,85.0,88.0,85.0,87.0,89.0,88.0,88.0,98.0,95.0,96.0,
                     98.0,87.0,97.0,96.0,96.0,98.0,97.0,96.0,96.0,96.0, 95.0]

        self.x = [-0.000332425059276, 1.0636889236, 2.13912070285, 3.00185047907, 3.88178605292, 4.65535102194,
                  5.3571315998, 6.26101716404, 7.28050003775, 8.24738282657, 9.31274168444, 10.037954879, 10.9566711011,
                  11.3643879805, 11.805139835, 12.4310122398, 13.2051231907, 11.8406849956, 10.7741211708, 9.4762986254,
                  9.24321061544, 9.32147428441, 7.89252291799, 6.7397220175, 6.374493178, 7.486751478971, 5.49702762734,
                  6.34267487075, 5.2790851546, 4.21913627418, 2.75753426428, 1.6265093785, 0.476690448964, -0.72123844,
                  -1.71869430597, -2.7439151126, -3.86449939746, -4.70359784112, -5.88819538431, -7.16207683588,
                  -8.6267529762, -9.8849200524, -11.3861023474, -11.8342977873, -13.7511491892, -16.3219998267,
                  -18.0272447492, -19.9056910309, -21.5546122316, -23.9620054817, -28.0205370402, -29.7800083068,
                  -30.8890115289, -22.4101569176, -15.838037536, -14.3960031497, -14.2548153562, -14.1927910478,
                  -14.3192646171, -14.4411979343,-12.129190896, -10.0082708278, -6.10902008203, -3.18872286608,
                  -0.552081111309, 0.076362208473, 2.07108726346, -15.047]

        self.y = [0, -0.0427079516637,-0.078456993529,-0.0948406465011,-0.100975794943,-0.0957378227573,-0.0940457287595,
                  -0.10428691588,-0.0124637620218,-0.0594008873481,-0.101911648449,-0.596882635839,-0.395106659561,
                  0.144615894979,0.386001336134,0.505914794083,0.344926805436,0.557267368159,-0.275279304565,
                  -1.09860342405,-2.24821363604,-3.21244542694,-4.61165973986,-5.49294192397,-7.7109137562119,
                  -10.0928374786778,-8.934299073831,-9.038041968679,-8.937964852007,-8.949919309199,-8.50147941754,
                  -8.6038801616,-8.7090676541,-8.84716783723,-8.63891410332,-8.85482089146,-9.04063685898,-9.1992528697,
                  -9.43021693835,-9.62005025291,-9.55222985525,-9.52080708764,-9.50438357814,-9.3269243194,
                  -9.57857667168,-9.44515093072,-9.3672957355,-9.24081543489,-9.25288321551,-9.34033221181,
                  -9.18755013489,-9.32934913882,-10.1291970289,-9.83148433157,-9.51531784213,-6.75844144072,
                  -4.58431348946,-1.7507607011,0.641350919596,2.61987475694,6.31999504949,6.3165971408,
                  6.3692069156982,6.3488310045,6.30364374939,2.68635061465,0.165081362707, 6.17]


        # Array containing intensities off radio signal

        self.distance_between_signals = False

        # Parameters variables

        # Marker variables
        self.radio_markers_array = MarkerArray()
        self.radio_markers_array.markers = []

        self.radio_new_marker = Marker()
        self.marker_counter = 1

        # Starting ROS
        rospy.init_node('radio_to_rviz', anonymous=True)
        rospy.loginfo("Starting radio signal mapping node...")

        # self.subscriber_radio = rospy.Subscriber("/radio/status", Float32MultiArray, self.subscriber_radio_callback)
        # self.subscriber_odom = rospy.Subscriber("/tf", TFMessage, self.odom_callback)

        self.radio_markers_publisher = rospy.Publisher('/radio_markers', MarkerArray, queue_size=10, latch=True)

        self.setting = termios.tcgetattr(sys.stdin)
        self.key = 0
        self.init_node()

    def init_node(self):
        self.publish_radio_signal_array()
        while not rospy.is_shutdown():
            rospy.sleep(rospy.Duration(10))

    def get_rssi_to_rgb(self, green_value):
        """
        Function to remap the radio signal value (from 0 to 100) to a value between  0 and 1
        """
        new_value = (((green_value - self.radio_min) * self.NewRange) / self.OldRange) + self.green_min
        return new_value

    def publish_radio_signal_array(self):

        for i in range(len(self.RSSI)):
            intensity = self.get_rssi_to_rgb(self.RSSI[i])

            self.radio_new_marker = Marker()
            self.radio_new_marker.header.frame_id = "map"
            self.radio_new_marker.header.stamp = rospy.Time.now()
            self.radio_new_marker.ns = "espeleo_radio_map"
            self.radio_new_marker.id = self.marker_counter
            self.radio_new_marker.type = Marker.CUBE
            self.radio_new_marker.action = Marker.ADD
            self.radio_new_marker.pose.orientation.w = 1.0
            self.radio_new_marker.pose.position.x = self.x[i]
            self.radio_new_marker.pose.position.y = self.y[i]
            self.radio_new_marker.pose.position.z = 0
            self.radio_new_marker.scale.x = 0.1
            self.radio_new_marker.scale.y = 0.1
            self.radio_new_marker.scale.z = 0.1

            self.radio_new_marker.color.a = 1.0
            self.radio_new_marker.color.g = 0.8 * (1 - 2.0 * abs(intensity - 1 / 2))
            self.radio_new_marker.color.r = 1.0 * intensity
            self.radio_new_marker.color.b = 1 - 1.0 * intensity
            self.marker_counter += 1
            self.radio_new_marker.lifetime = rospy.Duration(0)

            self.radio_markers_array.markers.append(self.radio_new_marker)
        self.radio_markers_publisher.publish(self.radio_markers_array)

    def getch(self, timeout=None):
        self.settings = termios.tcgetattr(sys.stdin)
        try:
            self.setup_term(sys.stdin)
            try:
                rw, wl, xl = select.select([sys.stdin], [], [], timeout)
            except select.error:
                return
            if rw:
                key = sys.stdin.read(1)

                if key == '\x1b':
                    key += sys.stdin.read(2)

                return key
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.setting)

    def setup_term(self, fd, when=termios.TCSAFLUSH):
        mode = termios.tcgetattr(fd)
        mode[tty.LFLAG] = mode[tty.LFLAG] & ~(termios.ECHO | termios.ICANON)
        termios.tcsetattr(fd, when, mode)


if __name__ == '__main__':
    radio_obj = RadioSignal()
