#include<ModbusMaster.h>

//HardwareSerial Serial6(PA12, PA11);
HardwareSerial Serial1(PA10, PA9);


#define MAX485_DE 4
#define MAX485_RE_NEG 5

ModbusMaster node;

void setup() {
  
  Serial.begin(9600);
//  Serial6.begin(9600);
  Serial1.begin(9600);

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  //My slave uses 9600 baud
  delay(10);
  Serial.println("starting arduino: ");
  Serial.println("setting up Serial ");
  Serial.println("setting up RS485 port ");
//  slave id
  node.begin(2, Serial1);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  uint8_t i, result;
  uint16_t data[2];
  union
  {
    uint32_t j;
    float f;
  }u;
  
//  result = node.readHoldingRegisters(156,2);
//  Serial.print("Response received : ");
//  Serial.println(result, HEX);

//  result = node.readHoldingRegisters(0,2);
  result = node.readHoldingRegisters(0,1);
  Serial.print("Response received : ");
  Serial.println(result, HEX);

  if (result == node.ku8MBSuccess) { 
//      for (i = 0; i < 2; i++)
//      {
//        data[i] = node.getResponseBuffer(i);
//      }
//      u.j=((unsigned long)data[1]<<16 |data[0]);
//      Serial.print("freq -> ");
//      Serial.println(u.f);

//      for (i = 0; i < 2; i++)
//      {
//        data[i] = node.getResponseBuffer(i);
//      }
//      u.j=((unsigned long)data[1]<<16 |data[0]);
//      int vall = (int16_t)(data[1] << 8 | data[0]);
      Serial.print("freq -> ");
      Serial.println(node.getResponseBuffer(0));
  }
  else if(result == node.ku8MBIllegalDataAddress) {
    Serial.println("Illegal Data address !");  
  }
  else if(result == node.ku8MBIllegalDataValue) {
    Serial.println("Illegal data value !");  
  }
  else if(result == node.ku8MBResponseTimedOut) {
    Serial.println("Response timeout !");  
  }

  Serial.print("\n");  

  delay(200);
}

// MODBUS LIB CALLBACK FUNCTIONS
void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}
