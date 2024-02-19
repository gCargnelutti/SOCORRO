#include <Arduino.h>
#include <port.h>

void setup() {
  
pinMode (S5, OUTPUT);

}

void loop() {
digitalWrite(S5,HIGH);
delay(1000);
digitalWrite(S5, LOW);
delay(1000);

}