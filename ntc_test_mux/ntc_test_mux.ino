#define ph_pin A0

#define mux_pin_1 5
#define mux_pin_2 6
#define mux_pin_3 7

int flag = 0;

#define pwm_pin A3

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
  switch(flag) {
    case 0 :
      analogWrite(pwm_pin, 64);
      break;
    case 1 :
      analogWrite(pwm_pin, 127);
      break;
    case 2 :
      analogWrite(pwm_pin, 192);
      break;
    case 3 :
      analogWrite(pwm_pin, 250);
      break;
  }

  Serial.print(" flag = "); Serial.println(flag);
  delay(100);
  
//  Serial.print("channel 1 : "); Serial.println((readMux(0)));
//  Serial.print("channel 2 : "); Serial.println((readMux(1)));
//  Serial.print("channel 3 : "); Serial.println((readMux(2)));
//  Serial.print("channel 4 : "); Serial.println((readMux(3)));
//  Serial.print("channel 5 : "); Serial.println((readMux(4)));
  int a = analogRead(0);
  float volt = (3.3/4096) * a;
  Serial.print("channel 5 : "); Serial.println(volt);

  Serial.println();

  flag += 1;
  if(flag > 3) flag = 0;
  
  delay(5000);
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

  digitalWrite(mux_pin_1, 0);
  digitalWrite(mux_pin_2, 0);
  digitalWrite(mux_pin_3, 0);
}

float getTemp(int val){ // SINGLE GET  SINGLE TEMPRATURE
  float voltageDividerR1 = 250;         // Resistor value in R1 for voltage devider method 
  float BValue = 4000;                    // The B Value of the thermistor for the temperature measuring range
  float R1 = 10000;                        // Thermistor resistor rating at based temperature (25 degree celcius)
  float T1 = 298.15f;                      /* Base temperature T1 in Kelvin (default should be at 25 degree)*/
  float R2 ;                              /* Resistance of Thermistor (in ohm) at Measuring Temperature*/
  float T2 ;                              /* Measurement temperature T2 in Kelvin */

  float a ;                               /* Use for calculation in Temperature*/
  float b ;                               /* Use for calculation in Temperature*/
  float c ;                               /* Use for calculation in Temperature*/
  float d ;                               /* Use for calculation in Temperature*/
  float e = 2.718281828f ; 
  
  R2 = (voltageDividerR1*val)/(4095-val);
  a = 1/T1;                                                                  
  b = log10(R1/R2);                                                         
  c = b / log10(e);                                                         
  d = c / BValue ;                                                           
  T2 = 1 / (a- d);    
  T2 = T2 - 273.15;
  
  return T2;
}
