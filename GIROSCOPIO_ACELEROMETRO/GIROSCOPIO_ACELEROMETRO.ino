// Transmissor

#include <SPI.h>
#include <LoRa.h>
#include<Wire.h>//Biblioteca para comunicação I2C
#include<math.h> //Biblioteca de Calculos matematicos

// Definição de pinos
#define pinRX 7
#define pintTX 8

// Definições gerais
#define alerta "Detectamos uma queda"
#define confirmacao "Alerta gerado"


// Declaração de constantes
const int MPU_addr=0x68; //Endereço do sensor
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //Variaveis para pegar os valores medidos
float AcnX, AcnY, AcnZ;
float Gx, Gy, Gz;
double arimaValue = 1.01;

// Funções de uso importante

double magnitudeVetorial (float x, float y, float z){
  double valor = sqrt(x*x + y*y + z*z);
  return valor;
}

float converteGravidade(int16_t valor, int16_t escala) {
    float newValor = (int)valor;
    float newEscala = (int)escala;
    float resultado = (newValor * newEscala )/ 32768.00;
    if (resultado <= 0){
      return (resultado * -1);
    }
    else{
      return resultado;
    }
}

double gravidadeParaAceleracao(float valor){
  return valor * 9.8;
}

double filtroArimaPrimeiraOrdem(double valor){
  arimaValue = 0.9 * arimaValue + (1 - 0.9) * valor;
  return arimaValue;
}

void setup(){

    // Testando se a comunicação está sendo feita
	while(!Serial){

    Serial.println("Transmissor LoRa");

	  if(!LoRa.begin(433E6)){  //Frequencia de Operacao (ou 915E6)
		  Serial.println("Falha ao Iniciar o Lora!");
		  while(1);
	
	  }

  }
  Wire.begin(); //Inicia a comunicação I2C
  Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
  Wire.write(0x6B); // registrador PWR_MGMT_1
  Wire.write(0); // Manda 0 e "acorda" o MPU 6050
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C); // Endereço do registrador de configuração do acelerômetro
  Wire.write(0x08); // Configurar para ±4g (veja o datasheet para outras opções)
  Wire.endTransmission(true);

  Serial.begin(9600); //Inicia a comunicaçao serial (para exibir os valores lidos)
	  
}

void loop(){

	LoRa.beginPacket(); // Inicia transmissão do pacote

  Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
  Wire.write(0x3B); // registrador dos dados medidos (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // faz um "pedido" para ler 14 registradores, que serão os registrados com os dados medidos
  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  AcnX = converteGravidade(AcX, 4); // Valor de X em g
  AcnY = converteGravidade(AcY, 4); // Valor de Y em g
  AcnZ = converteGravidade(AcZ, 4); // Valor de Z em g

  AcnX = gravidadeParaAceleracao(AcnX); // Valor de X em m/s²
  AcnY = gravidadeParaAceleracao(AcnY); // Valor de Y em m/s²
  AcnZ = gravidadeParaAceleracao(AcnZ); // Valor de Z em m/s²
  if( (AcnX > AcnY ) && (AcnX > AcnZ)){
    AcnX = AcnX - 10;
  }
  else if ((AcnY > AcnX ) && (AcnY > AcnZ)){
    AcnY = AcnY - 10;
  }
  else if ((AcnZ > AcnX ) && (AcnZ > AcnY)){
    AcnZ = AcnZ - 10;
  }

  double magnitude = magnitudeVetorial(AcnX, AcnY, AcnZ);

  double valorFinal =filtroArimaPrimeiraOrdem(magnitude);

	if(valorFinal >= 9.8 ){  //Condição que verifica se a queda ocorreu
 
		Serial.print("Queda Detectada = ");Serial.println(valorFinal);
		LoRa.print(confirmacao);
		
	}

	if(LoRa.available()){
		
		String recebido = LoRa.readString();
		Serial.println(recebido);

		if(recebido == confirmacao){
			LoRa.endPacket();
		}
	}

}