#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import requests

s = float()

def catcher():
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
    status.data = [0.0, 0.0, 0.0]
    #status.data[0] = 0.0 #s.quality = 0.0
    #status.data[1] = 0.0 #s.signal = 0.0
    #status.data[2] = 0.0 #s.noise = 0.0

    # Inicializacao do no que publica os status
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/radio/status', Float32MultiArray, queue_size=10)

    # Frequencia de atualizacao de 10hz (10 vezes por segundo)
    node_sleep_rate = rospy.Rate(10) 

    # Criacao do laco que garante a publicacao dos status ate que o no seja fechado
    while not rospy.is_shutdown():

        ws_path = rospy.get_param("ws_path")
        radio_ip = rospy.get_param("radio_ip")

        r = requests.get("http://{0}/status.cgi".format(radio_ip), auth=("ubnt", "espeleo"), verify= False)

        #r.encoding = 'utf-8'
        page = r.text.encode('utf-8')

        for line in page.splitlines():
            if 'quality' in line:
                matchedLine = line
                vector = matchedLine.split(',')
                for n in vector:
                    if 'quality' in n:
                        print(n)
                        match = n
            else:
                match = 0
        status.data[0] = filter(str.isdigit, match)

        for line in page.splitlines():
            if 'signal' in line:
                matchedLine = line
                vector = matchedLine.split(',')
                for n in vector:
                    if 'signal' in n:
                        print(n)
                        match = n
            else:
                match = 0
        status.data[1] = filter(str.isdigit, match)

        for line in page.splitlines():
            if 'noisef' in line:
                matchedLine = line
                vector = matchedLine.split(',')
                for n in vector:
                    if 'noisef' in n:
                        print(n)
                        match = n
        status.data[2] = filter(str.isdigit, match)

        # Publicando a variavel com os status
        pub.publish(status)
        node_sleep_rate.sleep()

if __name__ == '__main__':
    try:
        catcher()
    except rospy.ROSInterruptException:
        pass