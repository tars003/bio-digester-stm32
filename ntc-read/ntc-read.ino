int sensorPin = PA_0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  analogReadResolution(12);
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);

  Serial.print(sensorValue); Serial.print(" : ");

  sensorValue = getTemp(sensorValue);
  
  Serial.println(sensorValue);
  delay(200);
}

float getTemp(int val){
  
  float voltageDividerR1 = 267  ;         // Resistor value in R1 for voltage devider method 
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
  
  R2 = (voltageDividerR1*val)/(4096-val);
  a = 1/T1;                                                                  
  b = log10(R1/R2);                                                         
  c = b / log10(e);                                                         
  d = c / BValue ;                                                           
  T2 = 1 / (a- d);    
  T2 = T2 - 273.15;
  
  return T2;
}
