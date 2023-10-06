#!/usr/bin/env python3

import rospy
import re
import sys

from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from selenium import webdriver
from selenium.webdriver.chrome.service import Service
from selenium.webdriver.common.by import By
from selenium.webdriver.chrome.options import Options
from webdriver_manager.chrome import ChromeDriverManager
from bs4 import BeautifulSoup


class StatusReader:
    def __init__(self):
        # Starting variables
        self.status = Float32MultiArray()
        self.status.layout.data_offset = 0
        self.status.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        self.status.layout.dim[0].label = "lines"
        self.status.layout.dim[0].size = 1
        self.status.layout.dim[0].stride = 3
        # dim[1] is the horizontal dimension of your matrix
        self.status.layout.dim[1].label = "columns"
        self.status.layout.dim[1].size = 3
        self.status.layout.dim[1].stride = 3
        self.status.data = [0.0, 0.0, 0.0]

        self.user = 'ubnt'
        self.password = 'espeleo'

        if len(sys.argv) < 2:
            self.radio_ip = "192.168.1.23"
        else:
            self.radio_ip = sys.argv[1]

        if len(sys.argv) < 3:
            self.radio_pub_name = "/radio/status"
        else:
            self.radio_pub_name = "/radio/status_" + sys.argv[2] 


        # Starting ROS
        rospy.init_node('radio_status_catcher', anonymous=True)
        self.publisher = rospy.Publisher(self.radio_pub_name, Float32MultiArray, queue_size=10)
        self.sleep_rate = rospy.Rate(1)

        # Selenium configuration
        self.init_selenium()

    def init_selenium(self):
        #self.radio_ip = rospy.get_param('radio_ip')
        rospy.loginfo("Starting radio status cather node!")
        rospy.loginfo("Radio IP: %s", self.radio_ip )
        rospy.loginfo("Radio data: [quality(RSSI), Signal Strenght (-dB), Noise Floor (-dB)]")
        self.options = Options()
        self.options.add_argument('--headless')
        self.options.add_argument('--ignore-certificate-errors')
        self.options.add_argument('--allow-running-insecure-content')
        
        # Starting driver
        #try:
        self.driver = webdriver.Chrome(service=Service(ChromeDriverManager().install()),options=self.options)
        #except:
        #    rospy.logerr("Coudn't get chrome driver...")
        
        try:
            self.driver.get('http://' + self.radio_ip + '/status.cgi')
        except:
            rospy.logerr("Couldn't connect to the radio")
            rospy.logwarn("Radio IP not reachable. is %s the right IP?", self.radio_ip)
            

        # Main page
        username = self.driver.find_element(By.ID, "username")
        password = self.driver.find_element(By.ID, "password")

        # Sending credentials
        username.send_keys(self.user)
        password.send_keys(self.password)
        self.driver.find_element(By.CSS_SELECTOR, "input[type='submit']").click()

        while not rospy.is_shutdown():

            soup = BeautifulSoup(self.driver.page_source,"html5lib")
            page = []
            text = [i.getText() for i in soup]
            page.append(text)
            vector = str(page).split(",")

            self.status.data[0] = self.filter_str(vector[19])[0] # Quality 33 (RSSI 19) 
            self.status.data[1] = -self.filter_str(vector[18])[0] # Signal  (-Db)
            self.status.data[2] = -self.filter_str(vector[20])[0] # Noiseref
            self.publisher.publish(self.status)

            self.driver.refresh()

            self.sleep_rate.sleep()
            
    def filter_str(self, numb):
        return [int(match) for match in re.findall(r"\d+", numb)]


if __name__ == '__main__':
    radio_obj = StatusReader()