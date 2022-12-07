#include <IWatchdog.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <ModbusMaster.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>


HardwareSerial GSM_Serial(PA12, PA11);
HardwareSerial Serial1(PA10, PA9);
SoftwareSerial Serial_Uno(10, 11);
#define Serial_Mon Serial

LiquidCrystal_I2C lcd(0x27, 20, 4);
ModbusMaster node;
ModbusMaster node2;

#define DHTTYPE DHT11
#define DHTPIN D7
DHT dht(DHTPIN, DHTTYPE);

#define phPin1 0
#define LED1 D13
// HAS TO BE IN NUMERICAL FORMAT, CANNOT BE A0,A1,etc, USED FOR CALIBRATION ARR INDEXING
#define tempPin1 0
#define tempPin2 1
#define tempPin3 2
#define tempPin4 3
#define tempPin5 4


#define maxSerialLen 33
#define minSerialLen 27
#define deviceId 1

 
/* MODBUS 1 - ENEGY METER  */
#define MAX485_DE 3
#define MAX485_RE_NEG 4 
#define pingDelay 100              // PING DELAY FOR MODBUS AFTER EACH REQUEST
#define analogPingDelay 50         // DELAY BETWEEN ANALOG PINGS   
#define lcdMenuScreens 3           // TOTAL LCD SCREENS
#define maxPostTries 3             // MAX POST REQ TRIES
#define minModbusPingDelay  100     // DELAY BETWEEN SUBSEQUENT MODBUS PINGS IN ms

#define requestInterval 15000   // DATA UPDATE TIME ON THE SERVER
#define lcdMenuInterval 5000       // INTERCAL FOR MENU ROTATION
#define serialReadInterval 1000   // READ INTERVAL FROM UNO
#define lcdUpdateInterval 2500        // LCD REFRESH RATE
#define loopDelay 10            // MAIN LOOP DELAY
#define watchDogInterval 15000000 //  15 SECS WATCHDOG TIMER
#define initDealy 5000
#define tempThreshold 0.5
#define avgAnalogTime 10


bool debugLogs = false;

/* MODBUS 1 - ENEGY METER  */

/* pH SENSOR  */

float calibration_value = 29.84;
int phval = 0; 
unsigned long int avgval; 
int buffer_arr[10],temp;

/* pH SENSOR  */

/* GPRS CONFIG */
String apn = "airtelgprs.com";           
String apn_u = "";                 
String apn_p = "";             
// String url = "http://bio-digester-server.herokuapp.com/serial-data"; 
//String url = "https://bio-digester-monitor.onrender.com/serial-data";

String url = "http://ec2-3-109-201-138.ap-south-1.compute.amazonaws.com/serial-data"; 

String readData = ""; // STRING CONTAINING TIME AND GPIO STATUS TO BE SENT TO TH SERVER
unsigned long timer = 0;  // TIMER FOR MAKING POST REQUEST TO THE SERVER
unsigned long locationTimer = 0;  // TIMER FOR MAKING NEW LOCATION REQUEST AND UPDATING LOCATON
int incomingByte = 0; // USED TO STORE INCOMING BYTE, REPONSE TO AT COMMANDS FROM SIM800L
bool isReqSuccess = false;  // 200 HTTP CODE RECEIVED AFTER POST REQUEST
String locationRes = "";  // LOCATION REPOSNE STORED IN THIS AFTER AT+CNETSCAN
String espTime = "";  // TIME FROM SIM800L STORED IN THIS
bool startupFlag = 0; // USED TO TRIGGER LOCATION FROM SIM800, FOR THE VERY FIRST LOOP
int bearerFlag = 1; // STORIG AND OBSERVING bearer response
String bearerResponse = "";

#define WDT_TIMEOUT 180
#define defaultWaitTimeGSM 1000
#define locationInterval 240000
#define updateInterval 500
#define GPRS_INIT_DELAY 2500
#define gsm_init_delay 5000
/* GPRS CONFIG */

/* GLOBAL VARS */

float ph1, ph2, ph3, ph4, ph5;
String ph1Str, ph2Str, ph3Str, ph4Str, ph5Str;
float temp1, temp2, temp3, temp4, temp5, temp6;
float temp1Prev, temp2Prev, temp3Prev, temp4Prev, temp5Prev;
float rh;
float voltage, energy, flowRate, flowVolume;
float prevVoltage, prevEnergy, prevFlowRate, prevVolume;
/* GLOBAL VARS */

unsigned long menuTimer = 0;
unsigned long serialReadTimer = 0;
unsigned long lcdRefreshTimer = 0;
int lcdMenuScreen = 0;


bool newData = false;
String mainString = "";
String phString = "";

int reqFailCount = 0;

float calibArr[5] = {
  -16.55,
  -15.05,
  -15.78,
  -17.07,
  -16.55
};


void setup() {
  delay(initDealy);
  Init_Modbus1();
  
  GPIO_Init();
  dht.begin(); 
  
  Serial_Mon.begin(9600);
  Serial1.begin(9600);
  GSM_Serial.begin(9600);
  Serial_Uno.begin(9600);

  LCD_Init();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Configuring GPRS..."); 
  // INIT GPRS
  gsm_config_gprs();

  timer = millis();
  menuTimer = timer;
  serialReadTimer = timer;
  lcdRefreshTimer = timer;

  IWatchdog.begin(watchDogInterval);

  temp1Prev = getTemp(analogRead(tempPin1), tempPin1);  
  temp2Prev = getTemp(analogRead(tempPin2), tempPin2);  
  temp3Prev = getTemp(analogRead(tempPin3), tempPin3);  
  temp4Prev = getTemp(analogRead(tempPin4), tempPin4);  
  temp5Prev = getTemp(analogRead(tempPin5), tempPin5);   
}

void loop() {
  Serial.println("\n\nLoop start --------------------------------");
  unsigned long tt = millis();
  
  
  // IF POST REQ FAILED 3 TIMES, DONT RESET WATCHDOG TIMER
  if(reqFailCount >= maxPostTries) {
    Serial.print("\nPOST request fail count : ");
    Serial.println(reqFailCount);
    Serial.println();
    while(1){
      delay(1000);
      Serial.println("Waiting for restart");
    }
  }

  updateModbusData();
  get_all_temp();
  get_humidity();

  // FROM UNO
  if(millis() - serialReadTimer > serialReadInterval) {
    recvData();
    serialReadTimer = 0;
  }

  // MAKING A POST REQUEST TO THE SERVER
  if(millis() - timer > requestInterval ) {
    String processedData = formatString(deviceId, "5-11-22", "13:00:00", temp1, temp2, temp3, temp4, temp5, temp6, ph1Str, ph2Str, ph3Str, ph4Str, ph5Str, rh, energy, flowRate, flowVolume);
    Serial_Mon.print("Processed data : ");
    Serial_Mon.println(processedData);
    gsm_http_post(processedData);
    timer = millis();
  }

  // LCD MENU SCREEN UPDATE
  if(millis() - menuTimer > lcdMenuInterval) {
    lcdMenuScreen += 1;
    if(lcdMenuScreen >= lcdMenuScreens) lcdMenuScreen = 0;

    draw_menu(lcdMenuScreen);

    menuTimer = millis();
  }

//  if(millis() - lcdRefreshTimer > lcdUpdateInterval) {
//    // DRAW MENU ON LCD EVERY LOOP
//    draw_menu(lcdMenuScreen);
//
//    lcdRefreshTimer = millis();
//  }
  
  // MIN LOOP DELAY
  delay(loopDelay);
  // RELOAD WATCHDOG TIMER
  IWatchdog.reload();
  // LOOP TIME MEASUREMENT
  Serial.print("Loop end ------------------------------- time in millis : ");
  Serial.println(millis() - tt);
  Serial.println();
}

/* INITIALIZATION FUNCTIONS  */
void Init_Modbus1 (void) {     // INITIALIZE MODBUS FOR ENERGY METER
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  //  slave id
  node.begin(1, Serial1);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  //  slave id
  node2.begin(2, Serial1);
  node2.preTransmission(preTransmission);
  node2.postTransmission(postTransmission);
}
void GPIO_Init(void) {      // INIT GPOI
  // SET ANALOG READ RESOLUTION TO 12 BIT, 0-4095
  analogReadResolution(12);
  
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);
}
void LCD_Init(void) {              // INITIALIZE 20X4 LCD
  // INIT LCD
  lcd.begin();
  lcd.backlight();
  lcd.setBacklight(HIGH);
}
// FORMAT DATA TO BE SENT TO SERVER
String formatString(int mcId, String dateString, String timeString, float temp1, float temp2, float temp3, float temp4, float temp5, float temp6, String ph1, String ph2, String ph3, String ph4, String ph5, float rh, float energy, float flow, float flowVolume) {
    String processedData = "{";
    
    processedData += "\"mcId\":\"";
    processedData += mcId;    
    processedData += "\",";
    
    processedData += "\"date\":\"";        
    processedData += dateString;
    processedData += "\",";

    processedData += "\"time\":\"";        
    processedData += timeString;
    processedData += "\",";

    processedData += "\"temp1\":\"";        
    processedData += temp1;
    processedData += "\",";

    processedData += "\"temp2\":\"";        
    processedData += temp2;
    processedData += "\",";

    processedData += "\"temp3\":\"";        
    processedData += temp3;
    processedData += "\",";

    processedData += "\"temp4\":\"";        
    processedData += temp4;
    processedData += "\",";

    processedData += "\"temp5\":\"";        
    processedData += temp5;
    processedData += "\",";

    processedData += "\"temp6\":\"";        
    processedData += temp6;
    processedData += "\",";

    processedData += "\"ph1\":\"";        
    processedData += ph1;
    processedData += "\",";

    processedData += "\"ph2\":\"";        
    processedData += ph2;
    processedData += "\",";

    processedData += "\"ph3\":\"";        
    processedData += ph3;
    processedData += "\",";

    processedData += "\"ph4\":\"";        
    processedData += ph4;
    processedData += "\",";

    processedData += "\"ph5\":\"";        
    processedData += ph5;
    processedData += "\",";

    processedData += "\"rh\":\"";        
    processedData += rh;
    processedData += "\",";

    processedData += "\"energy\":\"";        
    processedData += energy;
    processedData += "\",";

    processedData += "\"flow\":\"";        
    processedData += flow;
    processedData += "\",";

    processedData += "\"flowVolume\":\"";        
    processedData += flowVolume;
    
    processedData += "\"}";
    return processedData;
}
/* INITIALIZATION FUNCTIONS  */

/* MODBUS FUNCTIONS  */
int getParameter(float *ans, int address, int len) {  // GET PARAMETER FROM ENERGY METER
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
    Serial_Mon.print("Response received : ");
    Serial_Mon.println(result, HEX);
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
        Serial_Mon.print("Float ans -> ");
        Serial_Mon.println(u.f);
      }
  }
  else if(result == node.ku8MBIllegalDataAddress) {
    if(debugLogs) {
        Serial_Mon.println("Illegal Data address !");  
    }
  }
  else if(result == node.ku8MBIllegalDataValue) {
    if(debugLogs) {
      Serial_Mon.println("Illegal data value !");  
    }
  }
  else if(result == node.ku8MBResponseTimedOut) {
    if(debugLogs) {
      Serial_Mon.println("Response timeout !");  
    }
  }
  return result;
}
int getParameter2( float *ans, int address, int len) { // GET PARAMETER FROM FLOW METER

  int result;
  
  // MAKING REQUEST TO SLAVE
  result = node2.readHoldingRegisters( address, len);
  if(debugLogs) {
    Serial_Mon.print("Response received : ");
    Serial_Mon.println(result, HEX);
  }
  // REQUEST TO SLAVE DONE

  
  if (result == node2.ku8MBSuccess) { 
      float flowRate = node2.getResponseBuffer(0);
      *ans = flowRate/10;
      
      if(false) {
        Serial_Mon.print("flow Rate  ans -> ");
        Serial_Mon.println(flowRate);
      }
  }
  else if(result == node2.ku8MBIllegalDataAddress) {
    if(debugLogs) {
        Serial_Mon.println("Illegal Data address !");  
    }
  }
  else if(result == node2.ku8MBIllegalDataValue) {
    if(debugLogs) {
      Serial_Mon.println("Illegal data value !");  
    }
  }
  else if(result == node2.ku8MBResponseTimedOut) {
    if(debugLogs) {
      Serial_Mon.println("Response timeout !");  
    }
  }
  return result;
}
int getParameter3( float *ans, int address, int len) { // GET PARAMETER FROM FLOW METER
  int result;

  // MAKING REQUEST TO SLAVE
  result = node2.readHoldingRegisters( 3, 2);
  if(false) {
    Serial_Mon.print("Response received : ");
    Serial_Mon.println(result, HEX);
  }
  // REQUEST TO SLAVE DONE

  
  if (result == node2.ku8MBSuccess) { 
      uint32_t a = ((unsigned long)node2.getResponseBuffer(1) << 16 | node2.getResponseBuffer(0));
      String b = String(a, HEX);
      int c = atoi(&b[0]);
      float d = (float)c / 1000;
      
//      Serial.println(d, 3);
      *ans = d;
  }
  else if(result == node2.ku8MBIllegalDataAddress) {
    if(debugLogs) {
        Serial_Mon.println("Illegal Data address !");  
    }
  }
  else if(result == node2.ku8MBIllegalDataValue) {
    if(debugLogs) {
      Serial_Mon.println("Illegal data value !");  
    }
  }
  else if(result == node2.ku8MBResponseTimedOut) {
    if(debugLogs) {
      Serial_Mon.println("Response timeout !");  
    }
  }
  return result;
}
void preTransmission(){ // MODBUS LIB CALLBACK FUNCTIONS
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}
void postTransmission(){ // MODBUS LIB CALLBACK FUNCTIONS
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}
/* MODBUS FUNCTIONS  */

/* LCD FUNCTIONS */
void draw_menu(int screen) {
  switch(screen) {
    case 0:
      draw_ph(ph1, ph2, ph3, ph4, ph5);
      break;
    case 1:
      draw_temp(temp1, temp2, temp3, temp4, temp5, temp6);
      break;
    case 2:
      draw_modbus_data(energy, flowRate, flowVolume, rh);
      break;
    default:
      draw_error();
  }
}
void draw_error(void) {
  lcd.clear();
  
  lcd.setCursor(0, 0); lcd.print("Something ");  
  lcd.setCursor(0, 1); lcd.print("ent wrong !");

}
void draw_ph(float ph1, float ph2, float ph3, float ph4, float ph5) {
  lcd.clear();
  
  lcd.setCursor(0, 0); lcd.print("pH1: "); lcd.print(ph1Str);
  lcd.setCursor(0, 1); lcd.print("pH2: "); lcd.print(ph2Str);
  lcd.setCursor(0, 2); lcd.print("pH3: "); lcd.print(ph3Str);
  lcd.setCursor(0, 3); lcd.print("pH4: "); lcd.print(ph4Str);
  lcd.setCursor(13, 0); lcd.print("pH5: "); lcd.setCursor(13, 1);  lcd.print(ph5Str);
}
void draw_temp(float temp1, float temp2, float temp3, float temp4, float temp5, float temp6) {
  lcd.clear();
  
  lcd.setCursor(0, 0); lcd.print("T1: "); lcd.print(temp1);
  lcd.setCursor(0, 1); lcd.print("T2: "); lcd.print(temp2);
  lcd.setCursor(0, 2); lcd.print("T3: "); lcd.print(temp3);
  lcd.setCursor(0, 3); lcd.print("T4: "); lcd.print(temp4);
  lcd.setCursor(12, 0); lcd.print("T5: "); lcd.setCursor(12, 1);  lcd.print(temp5);
  lcd.setCursor(12, 2); lcd.print("T6: "); lcd.setCursor(12, 3);  lcd.print(temp6);
}
void draw_modbus_data(float energy, float flowRate, float flowVolume, int rh) {
  lcd.clear();
  
  lcd.setCursor(0, 0); lcd.print("Kwh : "); lcd.print(energy);
  lcd.setCursor(0, 1); lcd.print("LPM : "); lcd.print(flowRate);
  lcd.setCursor(0, 2); lcd.print("RH %: "); lcd.print(rh);
  lcd.setCursor(0, 3); lcd.print("Vol : "); lcd.print(String(flowVolume, 3));
  
}
void draw_demo_screen(void) {
  // LCD PRINTS
  lcd.setCursor(0, 0); lcd.print("Volt: "); lcd.print(voltage);
  lcd.setCursor(0, 1); lcd.print("pH: "); lcd.print(ph1);
  lcd.print(" kWh: "); lcd.print(energy);
  lcd.setCursor(0, 2); lcd.print("Flow Rate :");  lcd.print(flowRate);  lcd.print(" lpm");
  lcd.setCursor(0, 3); lcd.print("Temp: "); lcd.print(temp1);  lcd.print(" Hum: 55");  
}
/* LCD FUNCTIONS */


/* MODBUS FUNCTIONS */
void updateModbusData(void) {
  prevVoltage = voltage;
  prevEnergy = energy;
  prevFlowRate = flowRate;
  prevVolume = flowVolume;

  voltage = get_voltage();
  energy = get_energy();
  flowRate = get_flow_rate();
  flowVolume = get_flow_volume();

  if(voltage == -1) voltage = prevVoltage;
  if(energy == -1) energy = prevEnergy;
  if(flowRate == -1) flowRate = prevFlowRate;
  if(flowVolume == -1) flowVolume = prevVolume;

  return;
}
float get_voltage(void) {
  int res;
  float vrPhase;
  res = getParameter(&vrPhase, 142, 2); // GETTING FREQ FROM 40143 address
  Serial_Mon.print("res: "); 
  Serial_Mon.print(res, HEX);
  Serial_Mon.print(" | vrPhase = "); 
  Serial_Mon.println(vrPhase);
  delay(pingDelay);

  if(isnan(vrPhase)) return -1;

  return vrPhase;
}
float get_energy(void) {
  int res;
  float kwh;
  res = getParameter(&kwh, 158, 2); // GETTING FREQ FROM 40159 address
  Serial_Mon.print("res: "); 
  Serial_Mon.print(res, HEX);
  Serial_Mon.print(" | kw = "); 
  Serial_Mon.println(kwh);
  delay(pingDelay);

  if(isnan(kwh)) return -1;

  return kwh;
}
float get_flow_rate(void) {
  int res;
  float flowRate;
  res = getParameter2(&flowRate, 0, 1);  // GETTING FLOW RATE FROM FLOW METER ADDRESS 0
  
  if(isnan(flowRate)) return -1;

  Serial_Mon.print("res: "); 
  Serial_Mon.print(res, HEX);
  Serial_Mon.print(" | Flow Rate  = "); 
  Serial_Mon.println(flowRate);

  delay(minModbusPingDelay);
  
  return flowRate;
}
float get_flow_volume(void) {
  int res;
  float flowVolume ;
  
  res = getParameter3(&flowVolume, 3, 2);  // GETTING FLOW RATE LSB FROM FLOW METER ADDRESS 4
  
  Serial_Mon.print("res: "); 
  Serial_Mon.print(res, HEX);
  Serial_Mon.print(" | Flow Volume  = "); 
  Serial_Mon.println(flowVolume);

  delay(minModbusPingDelay);

  if(isnan(flowVolume)) return -1;
  
  return flowVolume;
}
/* MODBUS FUNCTIONS */

/* TEMP / PH FUNCTIONS */
void get_humidity(void) {
  temp6 = dht.readTemperature();
  rh = dht.readHumidity();

  if (isnan(rh) || isnan(temp6)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    temp6 = 0;
    rh = 0;
    return;
  }
  return;
}
void get_all_ph(void) {
  ph1 = getPh(phPin1);
  delay(analogPingDelay);
  ph2 = getPh(phPin1);
  delay(analogPingDelay);
  ph3 = getPh(phPin1);
  delay(analogPingDelay);
  ph4 = getPh(phPin1);
  delay(analogPingDelay);
  ph5 = getPh(phPin1);
  
  return;
}
void get_all_temp(void) {
  float temp1Inst = getTemp(getAvg((tempPin1)), tempPin1 );  
  float temp2Inst = getTemp(getAvg((tempPin2)), tempPin2);  
  float temp3Inst = getTemp(getAvg((tempPin3)), tempPin3);  
  float temp4Inst = getTemp(getAvg((tempPin4)), tempPin4);  
  float temp5Inst = getTemp(getAvg((tempPin5)), tempPin5);   

//  float temp1Inst = getTemp((analogRead(tempPin1)));  
//  float temp2Inst = getTemp((analogRead(tempPin2)));  
//  float temp3Inst = getTemp((analogRead(tempPin3)));  
//  float temp4Inst = getTemp((analogRead(tempPin4)));  
//  float temp5Inst = getTemp((analogRead(tempPin5)));   
  
  if(1) {
    if(abs(temp1Inst - temp1Prev) > tempThreshold) temp1Prev = temp1Inst;
    temp1 = temp1Inst * 0.005 + temp1Prev * 0.995;
    temp1Prev = temp1;

    if(abs(temp2Inst - temp2Prev) > tempThreshold) temp2Prev = temp2Inst;
    temp2 = temp2Inst * 0.005 + temp2Prev * 0.995;
    temp2Prev = temp2;

    if(abs(temp3Inst - temp3Prev) > tempThreshold) temp3Prev = temp3Inst;
    temp3 = temp3Inst * 0.005 + temp3Prev * 0.995;
    temp3Prev = temp3;

    if(abs(temp4Inst - temp4Prev) > tempThreshold) temp4Prev = temp4Inst;
    temp4 = temp4Inst * 0.005 + temp4Prev * 0.995;
    temp4Prev = temp4;

    if(abs(temp5Inst - temp5Prev) > tempThreshold) temp5Prev = temp5Inst;
    temp5 = temp5Inst * 0.005 + temp5Prev * 0.995;
    temp5Prev = temp5;
  }

//  Serial.print("temps = ");
//  Serial.print(temp1);
//  Serial.print(",");
//  Serial.print(temp2);
//  Serial.print(",");
//  Serial.print(temp3);
//  Serial.print(",");
//  Serial.print(temp4);
//  Serial.print(",");
//  Serial.println(temp5);
  
  return;
}
float getPh(int analogPin) {     // GET SINGLE PH
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
  float volt=(float)avgval*5.0/4096;
  float ph_act = -5.70 * volt + calibration_value;
  
  return ph_act;
}
float getTemp(int val, int analogPin){ // SINGLE GET  SINGLE TEMPRATURE
  float voltageDividerR1 = 215;         // Resistor value in R1 for voltage devider method 
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

  T2 += calibArr[analogPin];
  
  return T2;
}
int getAvg(int analogPin) {
  // TAKE 10 SAMPLES
  for(int i=0;i<10;i++) 
  { 
    buffer_arr[i]=analogRead(analogPin);
    delay(avgAnalogTime);
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
/* TEMP / PH FUNCTIONS */


/* GSM FUNCTIONS */
// POST REQUEST
void gsm_http_post( String postdata) {
  // LED ON, INDICATING POST REQ HAS STARTED
  digitalWrite(LED1, HIGH);
  
  Serial_Mon.println(" --- Start GPRS & HTTP --- ");
  // gsm_send_serial("AT+SAPBR=1,1");
  // gsm_send_serial("AT+SAPBR=2,1");
  // gsm_send_serial("AT+HTTPINIT");
  // gsm_send_serial("AT+HTTPPARA=CID,1");
  // gsm_send_serial("AT+HTTPPARA=URL," + url);
  gsm_send_serial("AT+HTTPPARA=CONTENT,application/json");
  
  gsm_send_serial("AT+HTTPDATA=500,5000");
  gsm_send_serial(postdata);
  gsm_send_serial("AT+HTTPACTION=1");
  gsm_send_serial("AT+HTTPREAD"); 
  // gsm_send_serial("AT+HTTPTERM");
  // gsm_send_serial("AT+SAPBR=0,1");

  if(!isReqSuccess) {
    ledSignal(10);
  }
  else {
    ledSignal(2);      
  }
}
// GET GPS LOCATION FROM SIM800
void gsm_get_location(void) {
  Serial_Mon.println(" --- Start GT LOCATION--- ");
  gsm_send_serial("AT+CNETSCAN=1");
  gsm_send_serial("AT+CNETSCAN");
    
//  locationRes = formatLocationData(locationRes);
  Serial_Mon.println("/***************************************");    
  Serial_Mon.println("response code stored in string : " + locationRes);
  Serial_Mon.println("***************************************/");
}
// INITIALIZE GSM
void gsm_config_gprs() {
  Serial_Mon.println(" --- CONFIG GPRS --- ");
  // INDICATION GSM INITIALIZATION START
  digitalWrite(LED1, HIGH);

  delay(gsm_init_delay);

  while(bearerFlag) {
    gsm_send_serial("AT+SAPBR=3,1,Contype,GPRS");
    delay(GPRS_INIT_DELAY);
    gsm_send_serial("AT+SAPBR=3,1,APN," + apn);
    delay(GPRS_INIT_DELAY);
    gsm_send_serial("AT+SAPBR=0,1");
    delay(GPRS_INIT_DELAY);
    gsm_send_serial("AT+SAPBR=1,1");
    delay(GPRS_INIT_DELAY);   

    Serial.print("bearer respone -> ");
    Serial.println(bearerResponse);
    
    // BEARER NOT OK
    if(bearerResponse.indexOf("OK") == -1 || bearerResponse.length()<2) {
      bearerFlag = 1;
      Serial_Mon.println("Bearer Connection Failed !");
      ledSignal(15);
    }
    // BEARER OK
    else if(bearerResponse.indexOf("OK") != -1) {
      bearerFlag = 0;
      Serial_Mon.println("Bearer Connection Sucessful !");
      ledSignal(5);
    }
  }    

  digitalWrite(LED1, HIGH);

  gsm_send_serial("AT+SAPBR=2,1");
  delay(GPRS_INIT_DELAY);
  gsm_send_serial("AT+HTTPINIT");
  delay(GPRS_INIT_DELAY);
  gsm_send_serial("AT+HTTPPARA=CID,1");
  delay(GPRS_INIT_DELAY);
  gsm_send_serial("AT+HTTPPARA=URL," + url);

  // INDICATIING GSM INITIALIZATION HAS COMPLETED SUCCESFULLY  
  digitalWrite(LED1, LOW);
}

// SEND AT COMMANDS GSM
// HANDLES DIFFERENT WAITING TIMES FOR VARIOUS AT COMMANDS
void gsm_send_serial(String command) {
  IWatchdog.reload();
  
  Serial_Mon.println("Send ->: " + command + " len: " + command.length());
  GSM_Serial.println(command);
  long wtimer = millis();
  int waitTime = defaultWaitTimeGSM;
  String httpRes = "";
  
  // RESPONSE 200 IS RECEIVED IN THIS REQUEST
  if(command == "AT+HTTPACTION=1") waitTime = 15000;
  // ACTUAL JSON DATA
  else if(command[0] == '{') waitTime = 5000;
  // WAIT 30 SECS IF COMMAND IS INITIALIZE BEARING
  else if(command == "AT+SAPBR=1,1") waitTime = 30000;
  // WAIT 20 SECS IF COMMAND IS GPS SCAN
  if(command == "AT+CNETSCAN") {
    waitTime = 20000;
    locationRes = "";
  }
  // WAIT FOR RESPONSE FROM SIM800L
  while (wtimer + waitTime > millis()) {
    while (GSM_Serial.available()) {
      int incomingB ;      
      incomingB = GSM_Serial.read();
      Serial_Mon.write((char)incomingB);

      if(command == "AT+HTTPACTION=1") httpRes += String((char)incomingB);
      // UPDATE GPS DATA IN STRING
      else if(command == "AT+CNETSCAN") locationRes += String((char)incomingB);
      // UPDATE TIME IN STRING
      else if(command == "AT+CCLK?") espTime += String((char)incomingB);
      // UPDATE BEARER REPONSE IN STRING
      else if(command == "AT+SAPBR=1,1") bearerResponse += String((char)incomingB);
    }
    // IF BEARER CONNECTON SUCCESSFUL BEFORE 30 SECS, BREAK
    if(command == "AT+SAPBR=1,1") {
      if(bearerResponse.indexOf("OK") != -1 && bearerResponse.length()>2) break;
    }
    // IF POST REQUEST COMPLETE BEFORE 15 SECS, BREAK
    else if(command == "AT+HTTPACTION=1") {
      if(httpRes.indexOf("200") > 0) break;
    }
  }  
  Serial_Mon.println();

  // IF COMMAND WAS MAKE POST REQUEST, UPDATE REQ CODE (looking for code 200)
  if(command == "AT+HTTPACTION=1") {
    Serial_Mon.println("/***************************************");    
    Serial_Mon.println("response code stored in string : " + httpRes);
    Serial_Mon.println("***************************************/");
    // RESETTING THE WATCH DOG TIMER
    if(httpRes.indexOf("200") > 0) {
//      esp_task_wdt_reset();
      isReqSuccess = true;
      reqFailCount = 0;
    
      Serial.println("\nPOST request successfull !\n");
    }
    else  {
      isReqSuccess = false;
      reqFailCount += 1;
      Serial.print("\nPOST request failed ! Fail count : ");
      Serial.println(reqFailCount);
      Serial.println();
    }
  }
}
// TEST POST REQUEST AND WAIT FOR ALONG TIME
void test_post_request(void) {
  String a = "{ \"mcId\": \"2\", \"time\": \"22:49:00\", \"date\": \"4-11-22\", \"temp1\": \"25.5\", \"temp2\": \"25.5\", \"temp3\": \"25.5\", \"temp4\": \"25.5\", \"temp5\": \"25.5\", \"ph1\": \"7.2\", \"ph2\": \"7.2\", \"ph3\": \"7.2\", \"ph4\": \"7.2\", \"ph5\": \"7.2\", \"rh\": \"75\", \"energy\": \"50\", \"flow\": \"9\", \"flowVolume\": \"90\" }";
  gsm_http_post(a);
  delay(1000000);
}
void ledSignal(int count) {

  for(int i=0; i<count; i++) {
    digitalWrite(LED1, HIGH);
    delay(200);
    digitalWrite(LED1, LOW);
    delay(200);
  }
}
/* GSM FUNCTIONS */

// SERIAL READ FORM UNO
void recvData() {
  static boolean recvInProgress = false;
  static int ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

//  Serial_Mon.println("Inside recvData");

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
// CHECK DATA RECIVED FROM UNO
void parseNewData() {
    if (newData == true) {
        if(mainString.length()>maxSerialLen || mainString.length()<minSerialLen) {
          Serial_Mon.print("\n Corrrupt data!!! -> ");
          Serial_Mon.println(mainString);
          mainString = "";
        }
        else {
          phString = mainString;
          Serial.print("recvd serial data : ");
          Serial_Mon.println(phString);
          
          ph1Str = phString.substring(1,6);
          ph2Str = phString.substring(7,12);
          ph3Str = phString.substring(13,18);
          ph4Str = phString.substring(19,24);
          ph5Str = phString.substring(25,30);
          
        }
        newData = false;
    }
}
