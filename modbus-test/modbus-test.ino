#include <ModbusMaster.h>           

#define MAX485_DE      3
#define MAX485_RE_NEG  2

ModbusMaster node;             

void setup() {
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  pinMode(4,INPUT);
  pinMode(5,INPUT);

  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(115200);             //Baud Rate as 115200
  ModbusMaster(1, Serial);            //Slave ID as 1
  node.preTransmission(preTransmission);         //Callback for configuring RS-485 Transreceiver correctly
  node.postTransmission(postTransmission);
}

void loop() {
  float value = analogRead(A0);
  node.writeSingleRegister(0x40000,value);        //Writes value to 0x40000 holding register

  int a= digitalRead(4);                           //Reads state of push button 
  int b= digitalRead(5);

  if (a == 1) {
    node.writeSingleRegister(0x40001,1);      
  }

  else {
    node.writeSingleRegister(0x40001,0);              //Writes 0 to 0x40001 holding register
  }

  if (b == 1) {
    node.writeSingleRegister(0x40002,1);              //Writes 1 to 0x40002 holding register
  }

  else {
    node.writeSingleRegister(0x40002,0);                //Writes 0 to 0x40002 holding register
  }

}

void preTransmission()            //Function for setting stste of Pins DE & RE of RS-485
{
  digitalWrite(MAX485_RE_NEG, 1);             
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

