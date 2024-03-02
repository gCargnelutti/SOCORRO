#include <Arduino.h>
#include <port.h>
#include <matriz.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>


// sensor frontal QTR

QTRSensors qtr; 
const uint8_t SensorCount = 8;
uint16_t sensorValues [SensorCount];

BluetoothSerial SerialBT;


void comunicationBT(){
  
   if(SerialBT.available()){

    SerialBT.println("comando enviado");

    String valorRecebido = SerialBT.readString();
    
    Serial.println(valorRecebido);
    
    SerialBT.println (valorRecebido);

   

    if(valorRecebido == "estado_a" || valorRecebido == "Cliente Conectado!" || valorRecebido == "Cliente Desconectado!"){
      Serial.println(valorRecebido);
      SerialBT.print(valorRecebido);
    }

    else if(valorRecebido == "ok"){
      SerialBT.print(valorRecebido);
    }

    else if (valorRecebido == "a") {

      Serial.println("recebendo a calibração");
      SerialBT.print("recebendo a calibração");

      calibrar = HIGH;

    } 

    else if (valorRecebido == "b") {

      Serial.println("recebendo inciar percurso");
      SerialBT.print("recebendo inciar percurso");

      iniciar = HIGH;


    } 

    else if (valorRecebido == "c") {

      Serial.println("recebendo mapeamento");
      SerialBT.print("recebendo mapeamento");

       ledcWrite(A_channel, 0);
       ledcWrite(B_channel, 0);
      iniciar = LOW;

    } 

    else{
      
      Serial.println("recebendo pid");
      SerialBT.print("recebendo pid");

      String stgkP = valorRecebido.substring(2,9);
      double kP = stgkP.toDouble();
      
      String stgkI = valorRecebido.substring(10,17);
      double kI = stgkI.toDouble();
      
      String stgkD = valorRecebido.substring(18,25);
      double kD = stgkD.toDouble();

      String stgVel = valorRecebido.substring(26,29);
      double Vel = stgVel.toDouble();

      KP = kP;
      KD = kD;
      KI = kI;
      VEL = Vel * (1023/255);
      SerialBT.begin (VEL);


    }
  }


}


void setup() {

  Serial.begin(9600);
  SerialBT.begin ("Socorro real oficial");

  // PWM 

  ledcAttachPin(PWMA , A_channel);
  ledcSetup(A_channel , DEFAULT_LEDC_FREQ , _resolution);
  
  ledcAttachPin(PWMB , B_channel);
  ledcSetup(B_channel , DEFAULT_LEDC_FREQ , _resolution);

    ledcWrite(A_channel, 0);
    ledcWrite(B_channel, 0);
  
  // QTR

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){S1,S2,S3,S4,S5,S6,S7,S8}, SensorCount);
  //qtr.setEmitterPin(2); 

}

void loop() {

 comunicationBT();

 while (calibrar == HIGH){

 SerialBT.begin ("inicio da calibração");

  delay(100);



 for (uint16_t j = 0; j < 150; j++)
  {
        qtr.calibrate();
        delay(1);
  }



  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');

  }
  Serial.println();


   for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  

  delay(1000);

   qtr.read(sensorValues);

   for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  calibrar = LOW;

  SerialBT.begin ("fim da calibração");

 }


 while (iniciar == HIGH) {

    comunicationBT();
 
    qtr.read(sensorValues);

    uint16_t sensores [8];
    int16_t position = qtr.readLineWhite(sensores);
    int16_t setpoint = 3500;
    static int16_t lasterror = 0;
    int16_t error = position - setpoint;


    int16_t P = error;
    int16_t I = I + error;
    int16_t D = error - lasterror;

    lasterror = error;

    float PID = KP*P + KI*I + KD*D;

    int16_t motorA = VEL - PID;
    int16_t motorB = VEL + PID;

    ledcWrite(A_channel, motorA);
    ledcWrite(B_channel, motorB);

   // SENSOR LATERAL DIREITO PARA PARADA 

    bool readR = digitalRead (Sensor_R);

    if ( readR == HIGH && anteriorreadR == LOW ){

      contadorR ++;
      Serial.println (contadorR);

    }
    

    if (contadorR >= 12){

    ledcWrite(A_channel, 0);
    ledcWrite(B_channel, 0);

     iniciar = LOW;

     contadorR = 0;

    }

    anteriorreadR = readR;

  }
 
}
