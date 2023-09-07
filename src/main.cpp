#include <Arduino.h>
#include <QTRSensors.h>
#include <config.h>
#include <BluetoothSerial.h>
QTRSensors qtr;

BluetoothSerial SerialBT;

// QTR?
const uint8_t SensorCount = 8;
uint16_t sensorValues [SensorCount];


// BLUETOOTH
void comunicationBT(){
  if(SerialBT.available()){
    SerialBT.println("funfando");
    String valorRecebido = SerialBT.readString();
    Serial.println(valorRecebido);

    //formato do dado recebido
    //     kP     ki      kD
    // 000.000;000.000;000.000
    if(valorRecebido == "estado_a" || valorRecebido == "Cliente Conectado!" || valorRecebido == "Cliente Desconectado!"){
      // Serial.println(valorRecebido);
      SerialBT.print(valorRecebido);
    }
    else if(valorRecebido == "ok"){
      SerialBT.print(valorRecebido);
    }
    else{
      String stgkP = valorRecebido.substring(0,7);
      double kP = stgkP.toDouble();
      String stgkI = valorRecebido.substring(8,15);
      double kI = stgkI.toDouble();
      String stgkD = valorRecebido.substring(16,23);
      double kD = stgkD.toDouble();
      if((kP + kI + kD) < 100){
        Serial.println("parece que deu certo essa caralha aqui");
      }

      // prova real de que o numero está chegando inteiro

      SerialBT.print(kP, 6);
      SerialBT.print(" | ");
      SerialBT.print(kI, 6);
      SerialBT.print(" | ");
      SerialBT.println(kD, 6);
      SerialBT.print(kP);

    }
  }
}


// CALIBRACAO


void setup() {

  // PINMODES
  // PONTE H
  pinMode(Ain1, OUTPUT);
  pinMode(Ain2, OUTPUT);
  pinMode(Bin1, OUTPUT);
  pinMode(Bin2, OUTPUT);
  // IR SENSOR
  pinMode(IR, OUTPUT);
  digitalWrite(IR,HIGH);

 //CONFIGURACAOPWM 
  ledcAttachPin(PWMA , A_channel);
  ledcSetup(A_channel , DEFAULT_LEDC_FREQ , _resolution);
  

  ledcAttachPin(PWMB , B_channel);
  ledcSetup(B_channel , DEFAULT_LEDC_FREQ , _resolution);

  // CONFIGURACAO SENSORES
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){S1,S2,S3,S4,S5,S6,S7,S8}, SensorCount);
  qtr.setEmitterPin(2);

  // CONFIGURACAO PID
  int P = 0;
  int I = 0;
  int D = 0;




  // --------ROTINA DE CALIBRACAO--------
  delay(2000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);


 for (uint16_t j = 0; j < 150; j++)
  {
        qtr.calibrate();
        delay(1);
  }

  digitalWrite(LED_BUILTIN, LOW);

  // --------FIM DA CALIBRACAO--------


  SerialBT.begin("rodriguess")
  delay(5000);
 

}


void loop()
{
  comunicationBT();
  qtr.read(sensorValues);

for (uint8_t i = 0; i < SensorCount; i++)
  {

    Serial.print('\t');
  }
  Serial.println();

  delay(500);

  uint16_t sensores [8];
  int16_t position = qtr.readLineWhite(sensores); //AAAAAAAAAAAA!!!!!!
  int16_t setpoint = 3500;
  static int16_t lasterror = 0;
  int16_t error = position - setpoint;

  
  float KP = 0.05;
  float KI = 0;
  float KD = 0;
  int16_t P = error;
  int16_t I = I + error;
  int16_t D = error - lasterror;
  lasterror = error;
  float pidmano = KP*P;

  int16_t vel = 300;

  int16_t motorA = vel - pidmano;
  int16_t motorB = vel + pidmano;

  //Serial.println (P);
  //Serial.println (pidmano);
  //Serial.println (motorA);
  //Serial.println (motorB);

  ledcWrite(A_channel, 0); // MOTOR A ------
  ledcWrite(B_channel, 0); // MOTOR B ------
  digitalWrite(Ain1 , LOW);
  digitalWrite(Bin1 , LOW);
  digitalWrite(Ain2 , HIGH);
  digitalWrite(Bin2 , HIGH);

  delay(10);
}