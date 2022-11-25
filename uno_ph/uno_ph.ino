#include<avr/wdt.h>

#define phPin1 A0
#define phPin2 A1
#define phPin3 A2
#define phPin4 A3
#define phPin5 A5

#define loopInterval 1250
#define resetInterval 600000

float calibration_value = 28.4;
int phval = 0; 
unsigned long int avgval; 
int buffer_arr[10],temp;

float ph1, ph2, ph3, ph4, ph5;
char ph1String[5], ph2String[5], ph3String[5], ph4String[5], ph5String[5];
unsigned long loopTimer = 0;
unsigned long resetTimer = 0;


void setup() {
  Serial.begin(9600);
  delay(2000);
  Serial.println("Inside setup !!");

  wdt_disable();  
  delay(3000); 

  loopTimer = millis();
  resetTimer = millis();

  wdt_enable(WDTO_4S);  
}

void loop() {
  ph1 = getPh(phPin1);
  ph2 = getPh(phPin2);
  ph3 = getPh(phPin3);
  ph4 = getPh(phPin4);
  ph5 = getPh(phPin5);

  // ph1 = formatPh(ph1);
  // ph2 = formatPh(ph2);
  // ph3 = formatPh(ph3);
  // ph4 = formatPh(ph4);
  // ph5 = formatPh(ph5);

  if(millis() - loopTimer > loopInterval) {
    Serial.print("<");

    if(ph1 <= 9.99) Serial.print("0"); 
    Serial.print(ph1); Serial.print(",");
    if(ph2 <= 9.99) Serial.print("0"); 
    Serial.print(ph2); Serial.print(",");
    if(ph3 <= 9.99) Serial.print("0"); 
    Serial.print(ph3); Serial.print(",");
    if(ph4 <= 9.99) Serial.print("0"); 
    Serial.print(ph4); Serial.print(",");
    if(ph5 <= 9.99) Serial.print("0"); 
    Serial.print(ph5); Serial.print(",");
    
    Serial.print(">");
    Serial.println();
    
    loopTimer = millis();

    wdt_reset();
  }
  if(millis() - resetTimer > resetInterval) {
    while(1);        
  }    
  

}

float getPh(int analogPin) {     // GET SINGLE PH
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
  // CALCULATE PH VALUE FROM ANALOG VALUE
  float volt=(float)avgval*5.0/1024;
  float ph_act = -5.70 * volt + calibration_value;
  
  return ph_act;
}

float formatPh(float ph) {
  long val;   // ph = 123.456

  val = (long) (ph * 10L); // val = 1234
  ph= (float) val / 10.0; // x = 1234 / 10.0 = 123.4

  return ph;
}

// <11.20,11.10,11.10,11.20,13.00,>

