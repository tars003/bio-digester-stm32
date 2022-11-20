/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13 through 220 ohm resistor
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInput
*/

int sensorPin = A1;    // select the input pin for the potentiometer
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

  sensorValue = getTemp(sensorValue);
  
  Serial.println(sensorValue);
  delay(200);
}

float getTemp(int val){
  
  float voltageDividerR1 = 250;         // Resistor value in R1 for voltage devider method 
  float BValue = 4000;                    // The B Value of the thermistor for the temperature measuring range
  float R1 = 5000;                        // Thermistor resistor rating at based temperature (25 degree celcius)
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
