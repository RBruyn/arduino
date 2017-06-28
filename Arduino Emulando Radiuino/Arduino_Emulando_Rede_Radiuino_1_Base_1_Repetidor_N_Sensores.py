#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
* Autor: Prof. Dr. Omar C. Branquinho <omar.branquinho at gmail dot com>
* Orientado: Andre L. B. Déo <andredeo at gmail dot com>
* Junho 2017
Emula a rede com um repetidor e N nós sensores
Esse script determina o número de sensores
O ID da Base é 0
O ID do Repetidor é 20
'''

# PROGRAMA PARA LEITURA DOS SENSORES EMULADOS
import serial
import math
import time
import struct
from time import localtime, strftime

contador1 = 0
RSSIdS1 = RSSIdR1 = RSSIuB1 = RSSIuR1 = 0

# Configura a serial
n_serial = "/dev/ttyUSB0"
ser = serial.Serial(n_serial, 9600, timeout=0.5,parity=serial.PARITY_NONE) # seta valores da serial

# Identificação da base e Número de sensores
ID_base = raw_input('ID_base:')                 #byte 10 (esta informação é adicionada pela base, está aqui por questão didática)
ID_repetidor = raw_input('ID_repetidor:')       #byte 8
Num_Sensores = 3
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

##### LOOP INFINITO
while True:
	try:
		contador_tot1 = 0
		contador_pot1 = 0
		potmediad1 = 0.0
		potacumulad1 = 0.0
		potmeddbd1 = 0.0
		contador_err1 = 0
		potmediau1 = 0.0
		potacumulau1 = 0.0
		potmeddbu1 = 0.0
		PER1 = 0
		AcumDPd1 = 0
		AcumDPu1 = 0
		AcumVad1 = 0
		AcumVau1 = 0
		MedDPd1 = 0
		MedDPu1 = 0
		DPd1 = 0
		DPu1 = 0
      

		PotMaxd1 = -200
		PotMind1 = 10

		PotMaxu1 = -200
		PotMinu1 = 10
    
		# Imprime na tela o menu de opções
		print '1 - Mede Temperatura, Umidade e Luminosidade:'
		print 's - Para sair:'

		# Leitura da opção do menu escolhida
		Opcao = raw_input('Comando:')

		if Opcao == "1": # entra com a opção
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
					PacoteRX = ser.read(52) # faz a leitura de 52 bytes do buffer que recebe da serial pela COM

					# Checa se recebeu 52 bytes 
					if len(PacoteRX) == 52:


						rssidS1 = ord(PacoteRX[0]) # RSSI_DownLink_Sensor
						rssidR1 = ord(PacoteRX[1]) #RSSI_DownLink_Repetidor
						rssiuR1 = ord(PacoteRX[3]) #RSSI_UpLink_Repetidor
						rssiuB1 = ord(PacoteRX[2]) # RSSI_UpLink_Base

						#RSSI Downlink_Sensor - potência recebida pelo sensor
						if rssidS1 > 128:
							RSSIdS1=((rssidS1-256)/2.0)-74

						else:
							RSSIdS1=(rssidS1/2.0)-74

						#RSSI Downlink_Repetidor - potência recebida pelo repetidor da base
						if rssidR1 > 128:
							RSSIdR1=((rssidR1-256)/2.0)-81

						else:
							RSSIdR1=(rssidR1/2.0)-81

						#RSSI Downlink_Sensor - potência recebida pelo sensor
						if rssiuR1 > 128:
							RSSIuR1=((rssiuR1-256)/2.0)-81
               
						else:
							RSSIuR1=(rssiuR1/2.0)-81

						#RSSI Uplink_Base - potência recebida pela base
						if rssiuB1 > 128:
							RSSIuB1=((rssiuB1-256)/2.0)-81
                     
						else:
							RSSIuB1=(rssiuB1/2.0)-81


						# Leitura do AD0
						ad0t = ord(PacoteRX[16]) # tipo de sensor - no caso está medindo temperatura 
						ad0h = ord(PacoteRX[17]) # alto
						ad0l = ord(PacoteRX[18]) # baixo
						TEMP = float (ad0h * 256 + ad0l)/100	   
            
						# Leitura do AD1
						ad1t = ord(PacoteRX[19]) # tipo de sensor - no caso está medindo umidade
						ad1h = ord(PacoteRX[20]) # alto
						ad1l = ord(PacoteRX[21]) # baixo
						AD1 = float (ad1h * 256 + ad1l)/100
				
						# Leitura do AD2
						ad2t = ord(PacoteRX[22]) # tipo de sensor - no caso está medindo luminosidade
						ad2h = ord(PacoteRX[23]) # alto
						ad2l = ord(PacoteRX[24]) # baixo
						AD2 = float (ad2h * 256 + ad2l)/100			
				
				
						if RSSIdS1 > PotMaxd1:
							PotMaxd1 = RSSIdS1
               
						if RSSIdS1 < PotMind1:   
							PotMind1 = RSSIdS1

						if RSSIuB1 > PotMaxu1:
							PotMaxu1 = RSSIuB1
               
						if RSSIuB1 < PotMinu1:   
							PotMinu1 = RSSIuB1
               
               
						listaPotDesviod1[contador_pot1]= RSSIdS1  #Grava a potência de downlink para cálculo do desvio padrão
						listaPotDesviou1[contador_pot1]= RSSIuB1  #Grava a potência de uplink para cálculo do desvio padrão

						contador_pot1=contador_pot1+1 #incrementa o contador utilizado para a média de potência e para o desvio padrão
	
						potmwd1 = pow(10,(RSSIdS1/10))   #converte a potência de downlink em dBm para mW.
						potacumulad1 = potacumulad1 + potmwd1  #Soma a potência em mW em um acumulador

						potmwu1 = pow(10,(RSSIuB1/10))   #converte a potência de uplink em dBm para mW
						potacumulau1= potacumulau1 + potmwu1

               
						print 'SENSOR:', s + 1, 'LEITURA:', j + 1               
						print time.asctime(),'Temperatura', TEMP,'ºC','Umidade', AD1,'%', 'Luminosidade', AD2,'Lúmen', ' RSSIdS', RSSIdS1,'dBm', ' RSSIdR' , RSSIdR1,'dBm', ' RSSIuR', RSSIuR1,'dBm', ' RSSIuB' , RSSIuB1,'dBm'
						print >>S,time.asctime(),j,'Temperatura', TEMP,'ºC','Umidade', AD1,'%', 'Luminosidade', AD2,'Lúmen', ' RSSIdS', RSSIdS1,'dBm', ' RSSIdR' , RSSIdR1,'dBm', ' RSSIuR', RSSIuR1,'dBm', ' RSSIuB' , RSSIuB1,'dBm'
						time.sleep(int(TEMPO1))

            
					else: 
						contador_err1 = contador_err1 + 1
						print 'Perda de pacote'
						time.sleep(int(TEMPO1))

					contador_tot1 = contador_tot1 + 1
       
            
				#RELATÓRIO SENSOR #############
				if contador_pot1 == 0:
					contador_pot1 = 1

				for l in range(0,contador_pot1):
					AcumVad1 =AcumVad1+ listaPotDesviod1[l]   #acumula o valor da lista para calcular a média
					AcumVau1 =AcumVau1+ listaPotDesviou1[l]   #acumula o valor da lista para calcular a média

				MedDPd1 = float (AcumVad1)/float(contador_pot1)
				MedDPu1 = float (AcumVau1)/float(contador_pot1)

				for m in range(0,contador_pot1):
					AcumDPd1 =AcumDPd1+ pow((listaPotDesviod1[m]- MedDPd1),2)  #acumula o valor da variancia
					AcumDPu1 =AcumDPu1+ pow((listaPotDesviou1[m]- MedDPu1),2)  #acumula o valor da variancia

				DPd1 = float (AcumDPd1)/float(contador_pot1)   #termina o calculo da variancia
				DPu1 = float (AcumDPu1)/float(contador_pot1)   #termina o calculo da variancia

				potmediad1 = potacumulad1 /contador_pot1
         
				if potmediad1==0:
					potmediad1=0
				else:
					potmeddbd1 = 10*math.log10(potmediad1)
				
				print
				print 'RELATÓRIO SENSOR:', s + 1
				print 'A Potência média de Downlink em dBm foi:', potmeddbd1,' dBm'
				print 'A Potência Máxima de Downlink em dBm foi:', PotMaxd1,' dBm'
				print 'A Potência Mínima de Downlink em dBm foi:', PotMind1,' dBm'
				print 'O Desvio Padrão do sinal de Downlink foi:', DPd1
         
				print >>S,time.asctime()
				print >>S,time.asctime(),'RELATÓRIO SENSOR:', s + 1
				print >>S,time.asctime(),' A Potência média de Downlink em dBm foi:', potmeddbd1,' dBm'
				print >>S,time.asctime(),'A Potência Máxima de Downlink em dBm foi:', PotMaxd1,' dBm'
				print >>S,time.asctime(),'A Potência Mínima de Downlink em dBm foi:', PotMind1,' dBm'
				print >>S,time.asctime(),'O Desvio Padrão do sinal de Downlink foi:', DPd1


				potmediau1 = potacumulau1 /contador_pot1
				if potmediau1==0:
					potmediau1=0
				else:
					potmeddbu1 = 10*math.log10(potmediau1)
				
				print 'A Potência média de Uplink em dBm foi:', potmeddbu1,' dBm'
				print 'A Potência Máxima de Uplink em dBm foi:', PotMaxu1,' dBm'
				print 'A Potência Mínima de Uplink em dBm foi:', PotMinu1,' dBm'
				print 'O Desvio Padrão do sinal de Uplink foi:', DPu1
         
				print >>S,time.asctime(),' A Potência média de Uplink em dBm foi:', potmeddbu1,' dBm'
				print >>S,time.asctime(),'A Potência Máxima de Uplink em dBm foi:', PotMaxu1,' dBm'
				print >>S,time.asctime(),'A Potência Mínima de Uplink em dBm foi:', PotMinu1,' dBm'
				print >>S,time.asctime(),'O Desvio Padrão do sinal de Uplink foi:', DPu1  

				PER1 = (float(contador_err1)/float(contador_tot1))* 100
				print 'A PER foi de:', float(PER1),'%'
				print >>S,time.asctime(),'A PER foi de:', float(PER1),'%'
				print
				print >>S,time.asctime()
         
			S.close()
         
         
		else:
			#opção de saída
			ser.close() # fecha a porta COM
			print 'Fim da Execução'  # escreve na tela
			break 
          
			ser.flushInput()

	except KeyboardInterrupt:
		S.close()
		ser.close()
		break