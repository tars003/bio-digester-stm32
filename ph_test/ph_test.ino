#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

float calibration_value = 29.84;
int phval = 0; 

unsigned long int avgval; 
int buffer_arr[10],temp;

void setup() 
{
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("   Welcome to      ");
  lcd.setCursor(0, 1);
  lcd.print(" Circuit Digest    ");

  analogReadResolution(12);

  Serial.println("Inside setup !!");

  delay(2000);
  lcd.clear();
}
void loop() {
  for(int i=0;i<10;i++) { 
    buffer_arr[i]=analogRead(0);
    delay(30);
  }
  for(int i=0;i<9;i++){
    for(int j=i+1;j<10;j++){
      if(buffer_arr[i]>buffer_arr[j]){
        temp=buffer_arr[i];
        buffer_arr[i]=buffer_arr[j];
        buffer_arr[j]=temp;
      }
    }
  }

  avgval=0;
  for(int i=2;i<8;i++)
    avgval+=buffer_arr[i];
  avgval = avgval/6;
  
  float volt=(float)avgval*5/4096;
  float ph_act = -5.70 * volt + calibration_value;
  
  lcd.setCursor(0, 0);
  lcd.print("pH Val:");
  lcd.setCursor(8, 0);
  lcd.print(ph_act);

  Serial.print("pH -> ");
  Serial.print(ph_act);
  Serial.print(" analog val -> ");
  Serial.println(avgval);

  delay(1000);
}
