#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.firefox.options import Options
from bs4 import BeautifulSoup
#import time
#import socket

# Configuracao da porta a ser utilizada
#s = socket.socket()
#port = 12397 # Reserve a port for your service
#s.bind(('', port)) #Bind to the port
#s.connect(('', port))

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

	# Inicializacao do no que publica os status
	rospy.init_node('talker', anonymous=True)
	pub = rospy.Publisher('/radio/status', Float32MultiArray, queue_size=10)

	# Frequencia de atualizacao de 10hz (10 vezes por segundo)
	node_sleep_rate = rospy.Rate(10)

	# Criacao do laco que garante a publicacao dos status ate que o no seja fechado
	while not rospy.is_shutdown():
		# Pegando o parametro de caminho para workspace
		ws_path = rospy.get_param("ws_path")
		radio_ip = rospy.get_param("radio_ip")
		# Chamando o navegador firefox sem abrir uma janela
		options = Options()
		options.headless = True
		browser = webdriver.Firefox(options = options, executable_path = '{0}/src/espeleo_radio/scripts/geckodriver'.format(ws_path))
		# Definindo o url a ser acessado atraves do navegador aberto
		browser.get("http://{0}/status.cgi".format(radio_ip))
		#time.sleep(10)
		# Encontrando os campos onde serao escritos nome de usuario e senha
		username = browser.find_element_by_id("username")
		password = browser.find_element_by_id("password")
		# Colocando nome de usuario e senha nos campos apropriados
		username.send_keys("ubnt")
		password.send_keys("espeleo")
		# Clicando no botao de login
		login_attempt = browser.find_element_by_css_selector("input[type='submit']").click()
		# Extraindo o texto da pagina acessada
		soup = BeautifulSoup(browser.page_source,"html5lib")
		page = []
		text = [i.getText() for i in soup]
		page.append(text)
		vector = str(page).split(',')

		# Coletando as informacoes interessantes do texto extraido
		status.data[0] = float(filter(str.isdigit, vector[33]))
		status.data[1] = float(filter(str.isdigit, vector[18]))
		status.data[2] = float(filter(str.isdigit, vector[20]))

		# Fechando o navegador
		browser.quit()

		# Publicando a variavel com os status
		pub.publish(status)
		node_sleep_rate.sleep()

if __name__ == '__main__':
    try:
        catcher()
    except rospy.ROSInterruptException:
        pass