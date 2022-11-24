#define ph_pin 0

#define mux_pin_1 2
#define mux_pin_2 3
#define mux_pin_3 4

int muxChannel[8][3]={
  {0,0,0}, //channel 0
  {1,0,0}, //channel 1
  {0,1,0}, //channel 2
  {1,1,0}, //channel 3
  {0,0,1}, //channel 4
  {1,0,1}, //channel 5
  {0,1,1}, //channel 6
  {1,1,1}, //channel 7
};

void setup() 
{
  Serial.begin(115200);
  mux_init();
  analogReadResolution(12);  

  Serial.println("Inside setup !!");

  delay(2000);
}

void loop()
{
  Serial.print("channel 1 : "); Serial.println(readMux(0));
  Serial.print("channel 2 : "); Serial.println(readMux(1));
  Serial.print("channel 3 : "); Serial.println(readMux(2));
  Serial.print("channel 4 : "); Serial.println(readMux(3));
  Serial.print("channel 5 : "); Serial.println(readMux(4));

  Serial.println();
  delay(2000);
}

int readMux(int channel){

  digitalWrite(mux_pin_1, muxChannel[channel][0]);
  digitalWrite(mux_pin_2, muxChannel[channel][1]);
  digitalWrite(mux_pin_3, muxChannel[channel][2]);
  
  delay(100);
  
  int val = analogRead(ph_pin);
  return val;
}

void mux_init(void) {
  pinMode(mux_pin_1, OUTPUT);
  pinMode(mux_pin_2, OUTPUT);
  pinMode(mux_pin_3, OUTPUT);  
}
