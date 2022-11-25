#include <SoftwareSerial.h>

SoftwareSerial Serial_Uno(10, 11);

#define maxSerialLen 33
#define minSerialLen 27

bool newData = false;
String mainString = "";

void setup() {

  delay(1000);
  
  Serial.begin(9600);
  Serial_Uno.begin(9600);

}

void loop() {
  recvData();

}


void recvData() {
  static boolean recvInProgress = false;
  static int ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

//  Serial.println("Inside recvData");

  while(Serial_Uno.available() > 0 && newData == false) {
    rc = Serial_Uno.read();
    if(recvInProgress == true) {
      if (rc != endMarker) {
        mainString += rc;
      }
      else {
        mainString += rc;
        recvInProgress = false;
        ndx = 0;      
        newData = true;
      }
    }
    else if(rc == startMarker) {
      recvInProgress = true;
      mainString = "";                   // REALLY IMPORTANT
      mainString += rc;
    }
  }
  
  parseNewData();    
}

void parseNewData() {
    if (newData == true) {
        if(mainString.length()>maxSerialLen || mainString.length()<minSerialLen) {
          Serial.println("\n Corrrupt data!!!");
          Serial.println(mainString);
          Serial.println();
          mainString = "";
        }
        else {
          Serial.println(mainString);
        }
        newData = false;
    }
}
