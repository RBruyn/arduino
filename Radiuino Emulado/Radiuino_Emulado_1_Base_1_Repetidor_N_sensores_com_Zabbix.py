#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
* Autores *
Prof. Dr. Omar C. Branquinho <omar.branquinho at gmail dot com>
Ms. Raphael Montali da Assumpção <montali at gmail dot com>

* Orientado *
Andre L. B. Déo <andredeo at gmail dot com>

* Descrição *
Emula uma rede com comunicação via rádio, utilizando o protocolo Raiuino
Composta de 1 Base, 1 repetidor e N nós sensores.
O número de sensores é determinado nesse script.
O ID da Base é 0
O ID do Repetidor é 20

* Alterações *
- Julho 2017 - André Déo:
	- Script portado para utilização de Classes e Métodos
	- Função de envio de dados para o Zabbix
	- Função de leitura de dados de configuração do arquivo etc/config.conf

'''

# Importação de Bibliotecas e Módulos
import serial
import math
import time
import struct
from time import localtime, strftime
from ConfigParser import ConfigParser
from pyzabbix import ZabbixSender,ZabbixMetric

class RADIUINO(object):

	# Definições Gerais
	def __init__(self, config_file):

		print('[*] Obtendo Configuração ...')
		self.conf = dict()
		self.get_conf(config_file)
		print
		print('[+] Feito!\n')

		# Verifica se o envio para o Zabbix está habilitado
		if self.conf['zabbix_enable']:
			print '[+] Envio de dados para o Zabbix habilitado'
		print 

		print('[*] Programa Iniciado!\n')

	def get_conf(self, config_file):
		config = ConfigParser()
		config.read(config_file)

	# Definição das informações do Arquivo de Configuração
	def get_conf(self, config_file):
		config = ConfigParser()
		config.read(config_file)
		
		# Seção Sensor
		self.conf['serial'] = config.get('Sensor','serial')
						
		# Seção Zabbix
		self.conf['server_name'] = config.get('Zabbix','server_name')
		self.conf['agentd_config'] = config.get('Zabbix','agentd_config')
		self.conf['sensor_name'] = config.get('Zabbix','sensor_name').split(',')
		self.conf['zabbix_port'] = int (config.get('Zabbix','zabbix_port'))
		self.conf['zabbix_enable'] = config.getboolean('Zabbix','enable')
		self.conf['zabbix_keys'] = config.get('Zabbix','zabbix_keys').split()
		self.conf['zabbix_keys_rel'] = config.get('Zabbix','zabbix_keys_rel').split()
		
		return self.conf
	
	# Definição dos Parâmetros de Comunicação
	def radio(self):
		contador1 = 0
		RSSIdS1 = RSSIdR1 = RSSIuB1 = RSSIuR1 = 0

		# Configura a serial
		n_serial = self.conf['serial']
		ser = serial.Serial(n_serial, 9600, timeout=0.5,parity=serial.PARITY_NONE) # seta valores da serial

		# Identificação da base e Número de sensores
		ID_base = 0										 #byte 10 (esta informação é adicionada pela base, está aqui por questão didática)
		ID_repetidor = 20								 #byte 8
		Num_Sensores = len(self.conf['sensor_name'])
		ID_destino_volta = 0

		# Cria o vetor Pacote
		PacoteTX = {}
		PacoteRX = {}

		# Intervalo entre as medições
		TEMPO1 = 1

		#Cria o vetor para salvar os valores das potências
		listaPotDesviod1 ={}
		listaPotDesviou1 ={}

		# Cria Pacote de 52 bytes com valor zero em todas as posições
		for i in range(52): # faz um array com 52 bytes
			PacoteTX[i] = 0
			PacoteRX[i] = 0

		# Loop Infinito
		while True:
			try:
				contador_tot1 = 0
				contador_pot1 = 0
				potmediAD2 = 0.0
				potacumulAD2 = 0.0
				potmeddbd1 = 0.0
				contador_err1 = 0
				potmediau1 = 0.0
				potacumulau1 = 0.0
				potmeddbu1 = 0.0
				PER1 = 0
				AcumDPd1 = 0
				AcumDPu1 = 0
				AcumVAD2 = 0
				AcumVau1 = 0
				MedDPd1 = 0
				MedDPu1 = 0
				DPd1 = 0
				DPu1 = 0
			  	PotMaxd1 = -200
				PotMind1 = 10
				PotMaxu1 = -200
				PotMinu1 = 10
			
				# Imprime na tela o Menu de Opções
				print '[*] Opções do Programa:'
				print '1 - Mede Temperatura, Umidade e Luminosidade'
				print 's - Para sair'
				print

				# Leitura da opção escolhida no menu
				Opcao = raw_input('Comando:')

				if Opcao == "1": # Entra com a opção
					num_medidas = raw_input('Entre com o número de medidas = ')
					w = int(num_medidas)
				 
					for j in range(0,w):    #Inicializa uma lista para gravar as potências e calcular o desvio padrão
						listaPotDesviod1[j] = 0
						listaPotDesviou1[j] = 0
				
					Log = strftime("Coleta_de_dados_%Y_%m_%d_%H-%M-%S.txt")
					print "Arquivo de log: %s" % Log
					S = open(Log, 'w')
								
					for s in range(Num_Sensores):
						ID_destino_ida1 = s
						for j in range(w):
							# Limpa o buffer da serial
							ser.flushInput()

							#Contador de PacoteTX
							PacoteTX[13] = contador1 + 1
							PacoteTX[37] = 1 #Liga LDR
					
							# Coloca no pacote o ID_sensor e ID_base
							PacoteTX[8] = int(ID_repetidor)
							PacoteTX[9] = int(ID_destino_ida1)
							PacoteTX[10] = int(ID_base)
							PacoteTX[11] = int(ID_destino_volta)

							# TX pacote - envia pacote para a base transmitir
							for i in range(52):
								ser.write(chr(PacoteTX[i]))

							# Tempo de espera para que receba a resposta do sensor
							time.sleep(0.1)

							# RX pacote - recebe o pacote enviado pelo sensor
							PacoteRX = ser.read(52) # Faz a leitura de 52 bytes do buffer que recebe da serial pela COM

							# Checa se recebeu 52 bytes 
							if len(PacoteRX) == 52:


								rssidS1 = ord(PacoteRX[0]) # RSSI_DownLink_Sensor
								rssidR1 = ord(PacoteRX[1]) # RSSI_DownLink_Repetidor
								rssiuR1 = ord(PacoteRX[3]) # RSSI_UpLink_Repetidor
								rssiuB1 = ord(PacoteRX[2]) # RSSI_UpLink_Base

								# RSSI Downlink_Sensor - potência recebida pelo sensor
								if rssidS1 > 128:
									RSSIdS1=((rssidS1-256)/2.0)-74

								else:
									RSSIdS1=(rssidS1/2.0)-74

								# RSSI Downlink_Repetidor - potência recebida pelo repetidor da base
								if rssidR1 > 128:
									RSSIdR1=((rssidR1-256)/2.0)-81

								else:
									RSSIdR1=(rssidR1/2.0)-81

								# RSSI Downlink_Sensor - potência recebida pelo sensor
								if rssiuR1 > 128:
									RSSIuR1=((rssiuR1-256)/2.0)-81
					   
								else:
									RSSIuR1=(rssiuR1/2.0)-81

								# RSSI Uplink_Base - potência recebida pela base
								if rssiuB1 > 128:
									RSSIuB1=((rssiuB1-256)/2.0)-81
							 
								else:
									RSSIuB1=(rssiuB1/2.0)-81


								# Leitura do AD0
								ad0t = ord(PacoteRX[16]) # tipo de sensor - no caso está medindo temperatura 
								ad0h = ord(PacoteRX[17]) # alto
								ad0l = ord(PacoteRX[18]) # baixo
								AD1 = float (ad0h * 256 + ad0l)/100	   
					
								# Leitura do AD2
								AD2t = ord(PacoteRX[19]) # tipo de sensor - no caso está medindo umidade
								AD2h = ord(PacoteRX[20]) # alto
								AD2l = ord(PacoteRX[21]) # baixo
								AD2 = float (AD2h * 256 + AD2l)/100
						
								# Leitura do AD3
								AD3t = ord(PacoteRX[22]) # tipo de sensor - no caso está medindo luminosidade
								AD3h = ord(PacoteRX[23]) # alto
								AD3l = ord(PacoteRX[24]) # baixo
								AD3 = float (AD3h * 256 + AD3l)/100			
						
						
								if RSSIdS1 > PotMaxd1:
									PotMaxd1 = RSSIdS1
					   
								if RSSIdS1 < PotMind1:   
									PotMind1 = RSSIdS1

								if RSSIuB1 > PotMaxu1:
									PotMaxu1 = RSSIuB1
					   
								if RSSIuB1 < PotMinu1:   
									PotMinu1 = RSSIuB1
					   
					   
								listaPotDesviod1[contador_pot1]= RSSIdS1  # Grava a potência de downlink para cálculo do desvio padrão
								listaPotDesviou1[contador_pot1]= RSSIuB1  # Grava a potência de uplink para cálculo do desvio padrão

								contador_pot1=contador_pot1+1 # Incrementa o contador utilizado para a média de potência e para o desvio padrão
			
								potmwd1 = pow(10,(RSSIdS1/10))   # Converte a potência de downlink em dBm para mW.
								potacumulAD2 = potacumulAD2 + potmwd1  # Soma a potência em mW em um acumulador

								potmwu1 = pow(10,(RSSIuB1/10))   # Converte a potência de uplink em dBm para mW
								potacumulau1= potacumulau1 + potmwu1

					   
								print 'SENSOR:', s + 1, 'LEITURA:', j + 1               
								print time.asctime(),'Temperatura', AD1,'ºC','Umidade', AD2,'%', 'Luminosidade', AD3,'Lúmen', ' RSSIdS', RSSIdS1,'dBm', ' RSSIdR' , RSSIdR1,'dBm', ' RSSIuR', RSSIuR1,'dBm', ' RSSIuB' , RSSIuB1,'dBm'
								print >>S,'SENSOR:', s + 1, 'LEITURA:', j + 1
								print >>S,time.asctime(),j,'Temperatura', AD1,'ºC','Umidade', AD2,'%', 'Luminosidade', AD3,'Lúmen', ' RSSIdS', RSSIdS1,'dBm', ' RSSIdR' , RSSIdR1,'dBm', ' RSSIuR', RSSIuR1,'dBm', ' RSSIuB' , RSSIuB1,'dBm'
								time.sleep(int(TEMPO1))
													   
								# Define as variáveis do Script que alimentarão os itens no Zabbix
								L1=[AD1,AD2,AD3,RSSIdS1,RSSIdR1,RSSIuR1,RSSIuB1]
					
								# Faz um loop para enviar cada valor de métrica para cada item no Zabbix
								if self.conf['zabbix_enable']:
									try:
										x = 0
										while x<len(self.conf['zabbix_keys']):
											# Envia valores para o Zabbix
											z = self.conf['sensor_name'][s]
											metrics = []
											m = ZabbixMetric(z, self.conf['zabbix_keys'][x], L1[x])
											metrics.append(m)
											config_file = self.conf['agentd_config'] if self.conf['agentd_config'] else None
											zbx = ZabbixSender(zabbix_server=self.conf['server_name'], zabbix_port=self.conf['zabbix_port'], use_config=config_file)
											zbx.send(metrics)
											x += 1
											
									except Exception as error:
										print 'Problemas de comunicação com o Servidor Zabbix:', error
										print >>S,time.asctime(),'Problemas de comunicação com o Servidor Zabbix:', error
															
							else: 
								contador_err1 = contador_err1 + 1
								print 'Perda de pacote'
								time.sleep(int(TEMPO1))

							contador_tot1 = contador_tot1 + 1
						   
						# Relatório do Sensor
						if contador_pot1 == 0:
							contador_pot1 = 1

						for l in range(0,contador_pot1):
							AcumVAD2 =AcumVAD2+ listaPotDesviod1[l]   # Acumula o valor da lista para calcular a média
							AcumVau1 =AcumVau1+ listaPotDesviou1[l]   # Acumula o valor da lista para calcular a média

						MedDPd1 = float (AcumVAD2)/float(contador_pot1)
						MedDPu1 = float (AcumVau1)/float(contador_pot1)

						for m in range(0,contador_pot1):
							AcumDPd1 =AcumDPd1+ pow((listaPotDesviod1[m]- MedDPd1),2)  # Acumula o valor da variancia
							AcumDPu1 =AcumDPu1+ pow((listaPotDesviou1[m]- MedDPu1),2)  # Acumula o valor da variancia

						DPd1 = float (AcumDPd1)/float(contador_pot1)   # Termina o calculo da variancia
						DPu1 = float (AcumDPu1)/float(contador_pot1)   # Termina o calculo da variancia

						potmediAD2 = potacumulAD2 /contador_pot1
				 
						if potmediAD2==0:
							potmediAD2=0
						else:
							potmeddbd1 = 10*math.log10(potmediAD2)
						
						print
						print 'RELATÓRIO SENSOR:', s + 1
						print 'A Potência Média de Downlink em dBm foi:', potmeddbd1,' dBm'
						print 'A Potência Máxima de Downlink em dBm foi:', PotMaxd1,' dBm'
						print 'A Potência Mínima de Downlink em dBm foi:', PotMind1,' dBm'
						print 'O Desvio Padrão do sinal de Downlink foi:', DPd1
				 
						print >>S,time.asctime()
						print >>S,time.asctime(),'RELATÓRIO SENSOR:', s + 1
						print >>S,time.asctime(),' A Potência Média de Downlink em dBm foi:', potmeddbd1,' dBm'
						print >>S,time.asctime(),'A Potência Máxima de Downlink em dBm foi:', PotMaxd1,' dBm'
						print >>S,time.asctime(),'A Potência Mínima de Downlink em dBm foi:', PotMind1,' dBm'
						print >>S,time.asctime(),'O Desvio Padrão do sinal de Downlink foi:', DPd1

						potmediau1 = potacumulau1 /contador_pot1
						if potmediau1==0:
							potmediau1=0
						else:
							potmeddbu1 = 10*math.log10(potmediau1)
						
						print 'A Potência Média de Uplink em dBm foi:', potmeddbu1,' dBm'
						print 'A Potência Máxima de Uplink em dBm foi:', PotMaxu1,' dBm'
						print 'A Potência Mínima de Uplink em dBm foi:', PotMinu1,' dBm'
						print 'O Desvio Padrão do sinal de Uplink foi:', DPu1
				 
						print >>S,time.asctime(),' A Potência Média de Uplink em dBm foi:', potmeddbu1,' dBm'
						print >>S,time.asctime(),'A Potência Máxima de Uplink em dBm foi:', PotMaxu1,' dBm'
						print >>S,time.asctime(),'A Potência Mínima de Uplink em dBm foi:', PotMinu1,' dBm'
						print >>S,time.asctime(),'O Desvio Padrão do sinal de Uplink foi:', DPu1  

						PER1 = (float(contador_err1)/float(contador_tot1))* 100
						print 'A PER foi de:', float(PER1),'%'
						print >>S,time.asctime(),'A PER foi de:', float(PER1),'%'
						print
						print >>S,time.asctime()
														
						# Define as variáveis do Script que alimentarão os itens no Zabbix
						L2=[potmeddbu1,PotMaxu1,PotMinu1,DPu1,potmeddbd1,PotMaxd1,PotMind1,DPd1]
					
						# Faz um loop para enviar cada valor de métrica para cada item no Zabbix
						if self.conf['zabbix_enable']:
							try:
								x = 0
								while x<len(self.conf['zabbix_keys_rel']):
									# Envia valores para o Zabbix
									z = self.conf['sensor_name'][s]
									metrics = []
									m = ZabbixMetric(z, self.conf['zabbix_keys_rel'][x], L2[x])
									metrics.append(m)
									config_file = self.conf['agentd_config'] if self.conf['agentd_config'] else None
									zbx = ZabbixSender(zabbix_server=self.conf['server_name'], zabbix_port=self.conf['zabbix_port'], use_config=config_file)
									zbx.send(metrics)
									x += 1
									
							except Exception as error:
								print 'Problemas de comunicação com o Servidor Zabbix:', error
								print >>S,time.asctime(),'Problemas de comunicação com o Servidor Zabbix:', error
											
					S.close()
				 
				 
				else:
					# Opção de saída
					ser.close() # Fecha a porta COM
					print
					print '[*] Fim da Execução'  # Escreve na tela
					break 
				  
					ser.flushInput()

			except KeyboardInterrupt:
				S.close()
				ser.close()
				break
				
if __name__ == '__main__':
	radiuino = RADIUINO('etc/config.conf')
	radiuino.radio()