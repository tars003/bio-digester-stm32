#include <ModbusMaster.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

//HardwareSerial Serial6(PA12, PA11);
HardwareSerial Serial1(PA10, PA9);

LiquidCrystal_I2C lcd(0x27, 20, 4);

ModbusMaster node;


/* MODBUS 1 - ENEGY METER  */
#define MAX485_DE 4
#define MAX485_RE_NEG 5
#define pingDelay 500

#define phPin1 0

bool debugLogs = false;

/* MODBUS 1 - ENEGY METER  */

/* pH SENSOR  */

float calibration_value = 29.84;
int phval = 0; 
unsigned long int avgval; 
int buffer_arr[10],temp;

/* pH SENSOR  */



void setup() {
  delay(2000);
  Init_Modbus1();
  LCD_Init();
 
  Serial.begin(9600);
//  Serial6.begin(9600);
  Serial1.begin(9600);
  
}

void loop() {
  
  float freq;
  int res = getParameter(&freq, 156, 2); // GETTING FREQ FROM 40157 address
  Serial.print("res: "); 
  Serial.print(res, HEX);
  Serial.print(" | Freq = "); 
  Serial.println(freq);

  delay(pingDelay);

  float vrPhase;
  res = getParameter(&vrPhase, 142, 2); // GETTING FREQ FROM 40143 address
  Serial.print("res: "); 
  Serial.print(res, HEX);
  Serial.print(" | vrPhase = "); 
  Serial.println(vrPhase);

  delay(pingDelay);

  float vln;
  res = getParameter(&vln, 140, 2); // GETTING FREQ FROM 40141 address
  Serial.print("res: "); 
  Serial.print(res, HEX);
  Serial.print(" | VLN Avg = "); 
  Serial.println(vln);

  delay(pingDelay);

  float kwh;
  res = getParameter(&kwh, 158, 2); // GETTING FREQ FROM 40159 address
  Serial.print("res: "); 
  Serial.print(res, HEX);
  Serial.print(" | KWh = "); 
  Serial.println(kwh);

  float phValue = getPh(phPin1);

  delay(pingDelay);
  Serial.println('\n');

  // LCD PRINTS
  lcd.setCursor(0, 0); lcd.print("Volt: "); lcd.print(vrPhase);
  

  lcd.setCursor(0, 1); lcd.print("pH: "); lcd.print(phValue);
  lcd.print(" kWh: "); lcd.print(kwh);

  lcd.setCursor(0, 2); lcd.print("Flow Rate : 00");
  lcd.setCursor(0, 3); lcd.print("Temp: 25.9 Hum : 55%");
  
  delay(1000);
}

// INITIALIZE MODBUS FOR ENERGY METER
void Init_Modbus1 (void) {
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  //  slave id
  node.begin(1, Serial1);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

// INITIALIZE 20X4 LCD
void LCD_Init(void) {
  // INIT LCD
  lcd.begin();
  lcd.backlight();
  lcd.setBacklight(HIGH);
}


// GET PARAMETER FROM ENERGY METER
float getParameter(float *ans, int address, int len) {
  // CONVERT RESPONSE BUFFER TO FLOAT
  uint8_t i, result;
  uint16_t data[2];
  union
  {
    uint32_t j;
    float f;
  }u;

  // MAKING REQUEST TO SLAVE
  result = node.readHoldingRegisters( address, len);
  if(debugLogs) {
    Serial.print("Response received : ");
    Serial.println(result, HEX);
  }
  // REQUEST TO SLAVE DONE

  
  if (result == node.ku8MBSuccess) { 
      for (i = 0; i < 2; i++)
      {
        data[i] = node.getResponseBuffer(i);
      }
      u.j=((unsigned long)data[1]<<16 |data[0]); // CONVERTING RECEIVED BUFFER TO FLOAT
      *ans = u.f;                                 // STORING CONVERTED FLOAT TO RETURN FROM FUNCTION
      
      if(false) {
        Serial.print("Float ans -> ");
        Serial.println(u.f);
      }
  }
  else if(result == node.ku8MBIllegalDataAddress) {
    if(debugLogs) {
        Serial.println("Illegal Data address !");  
    }
  }
  else if(result == node.ku8MBIllegalDataValue) {
    if(debugLogs) {
      Serial.println("Illegal data value !");  
    }
  }
  else if(result == node.ku8MBResponseTimedOut) {
    if(debugLogs) {
      Serial.println("Response timeout !");  
    }
  }
  return result;
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


float getPh(int analogPin) {
  // TAKE 10 SAMPLES
  for(int i=0;i<10;i++) 
  { 
    buffer_arr[i]=analogRead(0);
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
  float volt=(float)avgval*5.0/1024;
  float ph_act = -5.70 * volt + calibration_value;
  
  return ph_act;
}
