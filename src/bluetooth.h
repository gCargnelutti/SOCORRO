#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

void comunicationBT(){
  if(SerialBT.available()){
    SerialBT.println("start");
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
        Serial.println("menor que 100");
      }

      // prova real de que o numero está chegando inteiro

      Serial.print(kP, 6);
      Serial.print(" | ");
      Serial.print(kI, 6);
      Serial.print(" | ");
      Serial.println(kD, 6);
      SerialBT.print(kP);

    }
  }
}
