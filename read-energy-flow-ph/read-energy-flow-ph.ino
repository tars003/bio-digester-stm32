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

#define LED1 D7

#define maxSerialLen 35

/* MODBUS 1 - ENEGY METER  */
#define MAX485_DE 3
#define MAX485_RE_NEG 5 
#define pingDelay 500              // PING DELAY FOR MODBUS AFTER EACH REQUEST
#define requestInterval 120000000      // DATA UPDATE TIME ON THE SERVER
#define analogPingDelay 50         // DELAY BETWEEN ANALOG PINGS   
#define lcdMenuInterval 5000       // INTERCAL FOR MENU ROTATION
#define lcdMenuScreens 3           // TOTAL LCD SCREENS

#define phPin1 0
#define tempPin1 1

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
String url = "http://bio-digester-server.herokuapp.com/serial-data"; 
// STRING CONTAINING TIME AND GPIO STATUS TO BE SENT TO TH SERVER
String readData = "";
// TIMER FOR MAKING POST REQUEST TO THE SERVER
unsigned long timer = 0;
// TIMER FOR MAKING NEW LOCATION REQUEST AND UPDATING LOCATON
unsigned long locationTimer = 0;
// USED TO STORE INCOMING BYTE, REPONSE TO AT COMMANDS FROM SIM800L
int incomingByte = 0;
// 200 HTTP CODE RECEIVED AFTER POST REQUEST
bool isReqSuccess = false;
// LOCATION REPOSNE STORED IN THIS AFTER AT+CNETSCAN
String locationRes = "";
// TIME FROM SIM800L STORED IN THIS
String espTime = "";
// USED TO TRIGGER LOCATION FROM SIM800, FOR THE VERY FIRST LOOP
bool startupFlag = 0;
// STORIG AND OBSERVING bearer response
int bearerFlag = 1;
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
float temp1, temp2, temp3, temp4, temp5, temp6;
int rh;
float voltage, energy, flowRate, flowVolume;
/* GLOBAL VARS */

unsigned long menuTimer = 0;
int lcdMenuScreen = 0;

bool newData = false;
String mainString = "";

void setup() {
  delay(2000);
  Init_Modbus1();
  LCD_Init();
  GPIO_Init();

  Serial_Mon.begin(9600);
  Serial1.begin(9600);
  GSM_Serial.begin(9600);
  Serial_Uno.begin(9600);

  // INIT GPRS
//  gsm_config_gprs();

  timer = millis();
  menuTimer = millis();
}

void loop() {

  voltage = get_voltage();
  energy = get_energy();
  flowRate = get_flow_rate();
//  get_all_ph();
//  get_all_temp();

  recvData();

  

  // MAKING A POST REQUEST TO THE SERVER
  if(millis() - timer > requestInterval ) {
    String processedData = formatString(1, "5-11-22", "13:00:00", temp1, temp2, temp3, temp4, temp5, ph1, ph2, ph3, ph4, ph5, 55, energy, flowRate, flowRate);
  
    Serial_Mon.print("Processed data : ");
    Serial_Mon.println(processedData);
    
    gsm_http_post(processedData);

    timer = millis();
  }

  if(millis() - menuTimer > lcdMenuInterval) {
    lcdMenuScreen += 1;
    if(lcdMenuScreen >= lcdMenuScreens) lcdMenuScreen = 0;

    menuTimer = millis();
  }
  
  draw_menu(lcdMenuScreen);
//  draw_ph(ph1, ph2, ph3, ph4, ph5);
    
  delay(500);
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
String formatString(int mcId, String dateString, String timeString, float temp1, float temp2, float temp3, float temp4, float temp5, float ph1, float ph2, float ph3, float ph4, float ph5, float rh, float energy, float flow, float flowVolume) {
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
float getParameter(float *ans, int address, int len) {  // GET PARAMETER FROM ENERGY METER
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
float getParameter2(float *ans, int address, int len) { // GET PARAMETER FROM FLOW METER

  int result;
  
  // MAKING REQUEST TO SLAVE
  result = node2.readHoldingRegisters( address, len);
  if(debugLogs) {
    Serial_Mon.print("Response received : ");
    Serial_Mon.println(result, HEX);
  }
  // REQUEST TO SLAVE DONE

  
  if (result == node2.ku8MBSuccess) { 
      int flowRate = node2.getResponseBuffer(0);
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
  
  lcd.setCursor(0, 0); lcd.print("pH1: "); lcd.print(ph1);
  lcd.setCursor(0, 1); lcd.print("pH2: "); lcd.print(ph2);
  lcd.setCursor(0, 2); lcd.print("pH3: "); lcd.print(ph3);
  lcd.setCursor(0, 3); lcd.print("pH4: "); lcd.print(ph4);
  lcd.setCursor(13, 0); lcd.print("pH5: "); lcd.setCursor(13, 1);  lcd.print(ph5);
}
void draw_temp(float temp1, float temp2, float temp3, float temp4, float temp5, float temp6) {
  lcd.clear();
  
  lcd.setCursor(0, 0); lcd.print("T1: "); lcd.print(temp1);
  lcd.setCursor(0, 1); lcd.print("T2: "); lcd.print(temp2);
  lcd.setCursor(0, 2); lcd.print("T3: "); lcd.print(temp3);
  lcd.setCursor(0, 3); lcd.print("T4: "); lcd.print(temp4);
  lcd.setCursor(13, 0); lcd.print("T5: "); lcd.setCursor(12, 1);  lcd.print(temp5);
  lcd.setCursor(13, 2); lcd.print("T6: "); lcd.setCursor(12, 3);  lcd.print(temp6);
}
void draw_modbus_data(float energy, float flowRate, float flowVolume, int rh) {
  lcd.clear();
  
  lcd.setCursor(0, 0); lcd.print("Energy(kwh): "); lcd.print(energy);
  lcd.setCursor(0, 1); lcd.print("LPM:         "); lcd.print(flowRate);
  lcd.setCursor(0, 2); lcd.print("Acc. Vol:    "); lcd.print(flowVolume);
  lcd.setCursor(0, 3); lcd.print("RH % :       "); lcd.print(rh);
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
float get_voltage(void) {
  int res;
  float vrPhase;
  res = getParameter(&vrPhase, 142, 2); // GETTING FREQ FROM 40143 address
  Serial_Mon.print("res: "); 
  Serial_Mon.print(res, HEX);
  Serial_Mon.print(" | vrPhase = "); 
  Serial_Mon.println(vrPhase);
  delay(pingDelay);

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

  return kwh;
}
float get_flow_rate(void) {
  int res;
  float flowRate;
  res = getParameter2(&flowRate, 0, 1);  // GETTING FLOW RATE FROM FLOW METER ADDRESS 0

  return flowRate;
}
float get_flow_volume(void) {
  int res;
  float flowVolume;
  res = getParameter2(&flowVolume, 0, 1);  // GETTING FLOW RATE FROM FLOW METER ADDRESS 0

  return flowVolume;
}
/* MODBUS FUNCTIONS */

/* TEMP / PH FUNCTIONS */
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
  temp1 = getTemp(analogRead(tempPin1));  
  delay(analogPingDelay);
  temp2 = getTemp(analogRead(tempPin1));  
  delay(analogPingDelay);
  temp3 = getTemp(analogRead(tempPin1));  
  delay(analogPingDelay);
  temp4 = getTemp(analogRead(tempPin1));  
  delay(analogPingDelay);
  temp5 = getTemp(analogRead(tempPin1));  
  delay(analogPingDelay);
  temp6 = getTemp(analogRead(tempPin1));

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
  float getTemp(int val){ // SINGLE GET  SINGLE TEMPRATURE
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
/* TEMP / PH FUNCTIONS */


/* GSM FUNCTIONS */
// POST REQUEST
void gsm_http_post( String postdata) {
  Serial_Mon.println(" --- Start GPRS & HTTP --- ");
  // gsm_send_serial("AT+SAPBR=1,1");
  // gsm_send_serial("AT+SAPBR=2,1");
  // gsm_send_serial("AT+HTTPINIT");
  // gsm_send_serial("AT+HTTPPARA=CID,1");
  // gsm_send_serial("AT+HTTPPARA=URL," + url);
  gsm_send_serial("AT+HTTPPARA=CONTENT,application/json");
  gsm_send_serial("AT+HTTPDATA=1200,5000");
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
    
    // BEARER NOT OK
    if(bearerResponse.indexOf("OK") != -1 || bearerResponse.length()<2) {
      bearerFlag = 0;
      Serial_Mon.print("Bearer Connection failed, resetting");
      ledSignal(5);
//      ESP.restart();
    }
  }    

  gsm_send_serial("AT+SAPBR=2,1");
  delay(GPRS_INIT_DELAY);
  gsm_send_serial("AT+HTTPINIT");
  delay(GPRS_INIT_DELAY);
  gsm_send_serial("AT+HTTPPARA=CID,1");
  delay(GPRS_INIT_DELAY);
  gsm_send_serial("AT+HTTPPARA=URL," + url);

  // INDICATIING GSM INITIALIZATION HAS COMPLETED SUCCESFULLY  
  digitalWrite(LED1, HIGH);
}
// SEND AT COMMANDS GSM
// HANDLES DIFFERENT WAITING TIMES FOR VARIOUS AT COMMANDS
void gsm_send_serial(String command) {
  Serial_Mon.println("Send ->: " + command + " len: " + command.length());
  GSM_Serial.println(command);
  long wtimer = millis();
  int waitTime = defaultWaitTimeGSM;
  String httpRes = "";
  
  // WAIT 5 SECS IF COMMAND IS EITHER POST REQUEST OR GARBAGE
  if(command == "AT+HTTPACTION=1" || command[0] == '{') waitTime = 5000;
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
    }
    else  {
      isReqSuccess = false;
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

void recvData() {
  static boolean recvInProgress = false;
  static int ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  Serial_Mon.println("Inside recvData");

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

void parseNewData() {
    if (newData == true) {

        if(mainString.length() > maxSerialLen) mainString = "";
        else Serial_Mon.println(mainString);
        newData = false;
        
    }
}
