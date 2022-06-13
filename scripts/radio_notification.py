#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Int16
import dynamic_reconfigure.client

quality = float()
signal = float()
noise = float()

def callback_status(data): 
    global quality, signal, noise
    quality = data.data[0]
    signal = data.data[1]
    noise = data.data[2]

def analyser():
    # Criando a variavel que contem os sinais de alerta
    w = Int16()

    # Inicializacao do no que publica os alertas
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/radio/warning', Int16, queue_size=10)
    
    # Subscrevendo no topico que publica os status do radio
    sub = rospy.Subscriber('/radio/status', Float32MultiArray, callback_status)

    # Frequencia de atualizacao de 10hz (10 vezes por segundo)
    node_sleep_rate = rospy.Rate(10) 

    # Criacao do laco que garante a publicacao dos alertas ate que o no seja fechado
    while not rospy.is_shutdown():

        # Tirando a diferenca entre noisef e signal
        dif = noise - signal

        # Pega os parametros de valores de referencia do arquivo launch 
        bq_r = float(rospy.get_param("borderline_quality_red"))
        bq_y = float(rospy.get_param("borderline_quality_yel"))
        bn = float(rospy.get_param("borderline_noise"))
        #bq = 30
        #bn = 35

        # Avisa quando o sinal fica com baixa qualidade
        if(quality <= bq_r):
            w = 2
            if(quality < 25):
                # Setting the return_finished parameter in the dynamic server
                rospy.set_param('/espeleo/return_active', True)
                dyn_reconfig_client.update_configuration({"return_active":True})
        elif(quality <= bq_y):
            w = 1
        else:
            w = 0

        # Publicando a variavel com os alertas
        pub.publish(w)
        node_sleep_rate.sleep()

if __name__ == '__main__':
    global dyn_reconfig_client
    # Handling the timeout when the dynamic reconfigure server is not available
    try:
        dyn_reconfig_client = dynamic_reconfigure.client.Client("espeleo", timeout=3)
    except rospy.ROSException as e:
        rospy.logwarn("dynamic reconfigure not found, continuing without it!")
        pass  
    try:
        analyser()
    except rospy.ROSInterruptException:
        pass