#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

float calibration_value = 29.84;
int phval = 0; 

unsigned long int avgval; 
int buffer_arr[10],temp;

#define ph_pin A0

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
  Serial.print("channel 1 : "); Serial.println(getPh(0));
  Serial.print("channel 2 : "); Serial.println(getPh(1));
  Serial.print("channel 3 : "); Serial.println(getPh(2));
  Serial.print("channel 4 : "); Serial.println(getPh(3));
  Serial.print("channel 5 : "); Serial.println(getPh(4));

  Serial.println();

  delay(5000);
}

float getPh(int analogPin) {     // GET SINGLE PH
  // TAKE 10 SAMPLES
  for(int i=0;i<10;i++) 
  { 
    buffer_arr[i]=readMux(analogPin);
    delay(30);
  }
  // SORT SAMPLES IN ASCENDING ORDER
  for(int i=0;i<9;i++)
  {
    for(int j=i+1;j<10;j++)
    {
      if(buffer_arr[i]>buffer_arr[j])
      {
        temp=buffer_arr[i];
        buffer_arr[i]=buffer_arr[j];
        buffer_arr[j]=temp;
      }
    }
  }  
  // TAKE AVERAGE OF 6 MIDDLE SAMPLES
  avgval=0;
  for(int i=2;i<8;i++)
    avgval+=buffer_arr[i];
  avgval = avgval/6;  
  // CALCULATE PH VALUE FROM ANALOG VALUE
  float volt=(float)avgval*5.0/4096;
  float ph_act = -5.70 * volt + calibration_value;
  
  return ph_act;
}

int readMux(int channel){

  digitalWrite(mux_pin_1, muxChannel[channel][0]);
  digitalWrite(mux_pin_2, muxChannel[channel][1]);
  digitalWrite(mux_pin_3, muxChannel[channel][2]);
  
  delay(10);
  
  int val = analogRead(ph_pin);
  return val;
}

void mux_init(void) {
  pinMode(mux_pin_1, OUTPUT);
  pinMode(mux_pin_2, OUTPUT);
  pinMode(mux_pin_3, OUTPUT);  
}
