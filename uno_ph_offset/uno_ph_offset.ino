int pH_Value1; 
int pH_Value2; 
int pH_Value3; 
int pH_Value4; 
int pH_Value5; 

float Voltage1;
float Voltage2;
float Voltage3;
float Voltage4;
float Voltage5;


void setup() 
{ 
  Serial.begin(9600);
} 
 
void loop() 
{ 
  pH_Value1 = analogRead(A0); 
  pH_Value2 = analogRead(A1); 
  pH_Value3 = analogRead(A2); 
  pH_Value4 = analogRead(A3); 
  pH_Value5 = analogRead(A4); 

  Voltage1 = pH_Value1 * (5.0 / 1023.0); 
  Voltage2 = pH_Value2 * (5.0 / 1023.0); 
  Voltage3 = pH_Value3 * (5.0 / 1023.0); 
  Voltage4 = pH_Value4 * (5.0 / 1023.0); 
  Voltage5 = pH_Value5 * (5.0 / 1023.0); 
  

  Serial.print("-------------------------------");
  Serial.print("-------------------------------");
  Serial.print("-------------------------------");
  // Serial.print("------------------------------");
  Serial.println("------------------------------");

  Serial.print(" | V1 = ");
  Serial.print(Voltage1); 
  Serial.print("  A1:");
  Serial.print(pH_Value1); 
  
  Serial.print(" | V2 = ");
  Serial.print(Voltage2); 
  Serial.print(" A2:");
  Serial.print(pH_Value2); 
  
  
  Serial.print(" | V3 = ");
  Serial.print(Voltage3); 
  Serial.print(" A3:");
  Serial.print(pH_Value3); 

  Serial.print(" | V4 = ");
  Serial.print(Voltage4); 
  Serial.print(" A4:");
  Serial.print(pH_Value4); 

  Serial.print(" | V5 = ");
  Serial.print(Voltage5); 
  Serial.print(" A5:");
  Serial.print(pH_Value5); 

  Serial.println();
  delay(1000); 
}