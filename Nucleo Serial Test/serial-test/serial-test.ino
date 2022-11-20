HardwareSerial Serial1(PA10, PA9);
HardwareSerial Serial6(PA12, PA11);

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial1.begin(9600);
  Serial6.begin(9600);
  
  Serial.println("Inside setup ! -> 0");    
  Serial2.println("Inside setup -> 2 !");    
  Serial1.println("Inside setup ! -> 1");    
  Serial6.println("Inside setup ! -> 6");    

  pinMode(13, OUTPUT);    

}

void loop() {
  Serial.println("freq  = 50.00-> 0");
  Serial2.println("freq  = 50.00 -> 2");
  Serial1.println("freq  = 50.00 -> 1");
  Serial6.println("freq  = 50.00 -> 6");
  delay(1000);
  digitalWrite(13, !digitalRead(13));

}
