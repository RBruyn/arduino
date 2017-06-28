/* Autor: Prof. Dr. Omar C. Branquinho <omar.branquinho at gmail dot com>
 * Orientado: Andre L. B. Déo <andredeo at gmail dot com>
 * Junho 2017
Emula a rede com um repetidor e N nós sensores
O script em Python determina o número de sensores

Endereçamento
                       Byte 8   Byte 9   Byte 10   Byte 11
Base para Repetidor    20       1        0         0
Repetidor para S1      1        1        20        0
S1 para Repetidor      20       0        1         1
Repetidor para Base    0        0        20        1

RSSI
Base para repetidor - byte 0
Repetidor para Sendor 1 - byte 1
Sensor 1 para Repetidor - byte 2
Rpetidor para Base - byte 3
 */

byte Pacote[52]; // Pacote de 52 bytes que será recebido pelo nó sensor

float temperatura,umidade,luminosidade; // Métricas do nó sensor

int contador_pacote_TX; // Variável para contar os pacotes transmitidos pelo nó sensor


void setup() 
{
  Serial.begin(9600);
  contador_pacote_TX = 0; // Variável de contador de pacotes transmitidos é zerado
}

void loop() 
{

// SEMPRE VAI RECEBER O PACOTE

  if (Serial.available() == 52)   // recebe pacote que entrou pela USB
  {
		
    // Leitura do buffer da serial e zera pacote de transmissão
     for (int i = 0; i < 52; i++) // Pacote[#] é preenchido com zero e PacoteRX[#] recebe os bytes do buffer
     {
	Pacote[i] = Serial.read();  // Faz a leitura dos bytes que estão no buffer da serial
        delay(1);
      }
		
      // ===================================================  Chega pacote da base para repetidor 
      if ( Pacote[8] == 20)
        {
          // Manda do repetidor 20 envia para o sensor X
          Pacote[10] = Pacote[8]; // O repetidor está enviando      		
          Pacote[8] = Pacote[9]; // Vai para o sensor X
          
          // Gera a RSSI - Range de -137 <- 0 -> -137,5
          Pacote[0] = random(-126,129);          
        }
      
      //  ==================================================  Pacote do repetidor para o sensor X
      if ( Pacote[8] == Pacote[9])
        {
          // Sensor manda pacote de volta para repetidor
          int endereco = Pacote[8];
          Pacote[8] = Pacote[10]; // O repetidor está enviando 
          Pacote[10] = endereco; // Vai para o sensor X
     	          
          // Gera Temperatura
          temperatura = 100*float(random (12,35));
          Pacote[16] = 0; // Pode ser utilizado para indicar o tipo de sensor no byte 16
          Pacote[17] = (byte) ((int)(temperatura)/256); // Valor inteiro no byte 17
          Pacote[18] = (byte) ((int)(temperatura)%256); // Resto da divisão no byte 18

          // Gera Umidade
          umidade = 100*float(random (0,100));          
          Pacote[19] = 0; // Pode ser utilizado para indicar o tipo de sensor no byte 19
          Pacote[20] = (byte) ((int)(umidade)/256); // Valor inteiro no byte 20
          Pacote[21] = (byte) ((int)(umidade)%256); // Resto da divisão no byte 21

          // Gera Luminosidade
          luminosidade = 100*float(random (0,40000));          
          Pacote[22] = 0; // Pode ser utilizado para indicar o tipo de sensor no byte 22
          Pacote[23] = (byte) ((int)(luminosidade)/256); // Valor inteiro no byte 23
          Pacote[24] = (byte) ((int)(luminosidade)%256); // Resto da divisão no byte 24

        
          // Gera a RSSI - Range de -137 <- 0 -> -137,5
          Pacote[1] = random(-126,129);
          //delay(1000);
        }

      // ===================================================  Chega pacote do sensor para a base 
      if ( Pacote[8] == 20)
        {
          // Manda do repetidor 20 envia para o sensor X
          Pacote[10] = Pacote[8]; // O repetidor está enviando          
          Pacote[8] = Pacote[9]; // Vai para o sensor X
          
          // Gera a RSSI - Range de -137 <- 0 -> -137,5
          Pacote[0] = random(-126,129);          
        }
 
        // Transmissão do pacote do repetidor para a base
        for (int i = 0; i < 52; i++)
          {
           Serial.write(Pacote[i]);
          }
        			

  }
}		
