#include<avr/wdt.h>

#define phPin1 0
#define phPin2 1
#define phPin3 2
#define phPin4 3
#define phPin5 4

#define loopInterval 1250
#define resetInterval 600000

#define phThreshold 0.5
#define avgAnalogTime 15

float calibArr[5] = {
  27.32,
  27.20,
  26.47,
  27.24,
  27.23
};

float calibration_value = 28.44;
int phval = 0; 
unsigned long int avgval; 
int buffer_arr[10],temp;

float ph1, ph2, ph3, ph4, ph5;
float prevPh1, prevPh2, prevPh3, prevPh4, prevPh5;


unsigned long loopTimer = 0;
unsigned long resetTimer = 0;
unsigned long testTimer = 0;


void setup() {
  Serial.begin(9600);
  delay(2000);
  Serial.println("Inside setup !!");

  wdt_disable();  
  delay(3000); 

  loopTimer = millis();
  resetTimer = millis();
  

  

  prevPh1 = getPh(phPin1);
  prevPh2 = getPh(phPin2);
  prevPh3 = getPh(phPin3);
  prevPh4 = getPh(phPin4);
  prevPh5 = getPh(phPin5);

  wdt_enable(WDTO_4S);  
}

void loop() {

  // testTimer = millis();  
  
  float insPh1 = getPh(phPin1);
  float insPh2 = getPh(phPin2);
  float insPh3 = getPh(phPin3);
  float insPh4 = getPh(phPin4);
  float insPh5 = getPh(phPin5);

  // PREV VALUE FILTER
  if(1) {
    if(abs(insPh1 - prevPh1) > phThreshold) prevPh1 = insPh1;
    ph1 = insPh1 * 0.010 + prevPh1 * 0.990;
    prevPh1 = ph1;

    if(abs(insPh2 - prevPh2) > phThreshold) prevPh2 = insPh2;
    ph2 = insPh2 * 0.010 + prevPh2 * 0.990;
    prevPh2 = ph2;

    if(abs(insPh3 - prevPh3) > phThreshold) prevPh3 = insPh3;
    ph3 = insPh3 * 0.010 + prevPh3 * 0.990;
    prevPh3 = ph3;

    if(abs(insPh4 - prevPh4) > phThreshold) prevPh4 = insPh4;
    ph4 = insPh4 * 0.010 + prevPh4 * 0.990;
    prevPh4 = ph4;

    if(abs(insPh5 - prevPh5) > phThreshold) prevPh5 = insPh5;
    ph5 = insPh5 * 0.010 + prevPh5 * 0.990;
    prevPh5 = ph5;
  }  
  
  // SEND SERIAL DATA
  if(millis() - loopTimer > loopInterval) {
    Serial.print("<");

    if(ph1 <= 9.99) Serial.print("0"); 
    Serial.print(abs(ph1)); Serial.print(",");
    if(ph2 <= 9.99) Serial.print("0"); 
    Serial.print(abs(ph2)); Serial.print(",");
    if(ph3 <= 9.99) Serial.print("0"); 
    Serial.print(abs(ph3)); Serial.print(",");
    if(ph4 <= 9.99) Serial.print("0"); 
    Serial.print(abs(ph4)); Serial.print(",");
    if(ph5 <= 9.99) Serial.print("0"); 
    Serial.print(abs(ph5)); Serial.print(",");
    
    Serial.print(">");
    Serial.println();
    
    loopTimer = millis();

    wdt_reset();
  }
  // RESET AFTER SET INTERVAL, STOPS UPDATING WATCHDOG TIMER
  if(millis() - resetTimer > resetInterval) {
    while(1);        
  }    

  // Serial.print("Time taken by loop : ");
  // Serial.println(millis() - testTimer);

}

float getPh(int analogPin) {     // GET SINGLE PH

  // testTimer = millis();
  avgval = getAvg(analogPin);
  // avgval = smooth(analogPin);
  // Serial.print("Time taken by avg method : ");
  // Serial.println(millis() - testTimer);
    

  // CALCULATE PH VALUE FROM ANALOG VALUE
  float volt=(float)avgval*5.0/1024;
  float ph_act = -5.70 * volt + calibArr[analogPin];
  
  delay(10);

  wdt_reset();
  return ph_act;
}

int getAvg(int analogPin) {
  // TAKE 10 SAMPLES
  for(int i=0;i<10;i++) 
  { 
    buffer_arr[i]=analogRead(analogPin);
    delay(15);
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

  return avgval;  
}

int smooth(int analogPin){
  int i;
  int value = 0;
  int numReadings = 30;

  for (i = 0; i < numReadings; i++){
    // Read light sensor data.
    value = value + analogRead(analogPin);

    // 1ms pause adds more stability between reads.
    delay(5);
  }

  // Take an average of all the readings.
  value = value / numReadings;

  // Scale to 8 bits (0 - 255).
  // value = value / 4;

  return value;
}

float formatPh(float ph) {
  long val;   // ph = 123.456

  val = (long) (ph * 10L); // val = 1234
  ph= (float) val / 10.0; // x = 1234 / 10.0 = 123.4

  return ph;
}

// <11.20,11.10,11.10,11.20,13.00,>

