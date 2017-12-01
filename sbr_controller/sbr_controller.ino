#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include "Nunchuk.h"

LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
SoftwareSerial bts(2, 3);
void setup()
{
  bts.begin(115200);
  Serial.begin(115200);
  Wire.begin();
  nunchuk_init();

  lcd.begin(16,2);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("init");
}
/*
 *  TUNING OPTIONS
 * 0: none
 * 1: pitch
 * 2: vel
 */
int tuning = 2;
char s[50];
int8_t jx = 0, jy = 0;
uint8_t bc = 0, bz = 0;
unsigned long lastT = 0;
char inStr[150] = "";
byte packet[11];
void loop()
{
  double kp = 0;
  double ki = 0;
  int16_t kd = 0;
  if(tuning == 1) {
    kp = analogRead(A0) * (180.0/1022.0);
    ki = analogRead(A1) * (2.3/1022.0);
    kd = analogRead(A2) * (4000.0/1022.0);
  } else if(tuning == 2) {
    kp = analogRead(A0) * (6.0/1022.0);
    ki = analogRead(A1) * (0.03/1022.0);
    kd = analogRead(A2) * (50.0/1022.0);
  }
  int frac_ki = (int)((ki - (int)ki) * 10000);
  int frac_kp = (int)((kp - (int)kp) * 100);
  if(millis() > lastT + 200) {
    char kp0s[5] = "";
    if(tuning == 1) {
      if(kp < 10) {
        strcpy(kp0s, "00");
      } else if(kp < 100) {
        strcpy(kp0s, "0");
      }
    }
    char kd0s[5] = "";
    if(kd < 10) {
      strcpy(kd0s, "000");
    } else if(kd < 100) {
      strcpy(kd0s, "00");
    } else if(kd < 1000) {
      strcpy(kd0s, "0");
    }
    char strfkp[20] = "";
    if(tuning == 2) {
      strcat(strfkp, ".");
      if(frac_kp < 10) strcat(strfkp, "0");
      char strfkp_unsc[10];
      itoa(frac_kp, strfkp_unsc, 10);
      strcat(strfkp, strfkp_unsc);
    }
    char strfki[20] = ".";
    if(frac_ki < 10) {
      strcat(strfki, "000");
    } else if (frac_ki < 100) {
      strcat(strfki, "00");
    } else if(frac_ki < 1000) {
      strcat(strfki, "0");
    }
    char strfki_unsc[10];
    itoa(frac_ki, strfki_unsc, 10);
    strcat(strfki, strfki_unsc);
    sprintf(s, "p %s%d%s i %d%s   d %s%d t %d", kp0s, (int)kp, strfkp, (int)ki, strfki, kd0s, kd,
    (int)((-4.6 - (jy/15.0)) * 100));
    char line1[17] = "";
    char line2[17] = "";
    for(int i = 0; i < sizeof(s)/sizeof(s[0]) - 1 && s[i] != '\0'; i++) {
      if(i < 16) {
        line1[i] = s[i];
        line1[i+1] = '\0';
      } else if(i < 32) {
        line2[i - 16] = s[i];
        line2[i - 15] = '\0';
      }
    }
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(line1);
    lcd.setCursor(0,1);
    lcd.print(line2);
    lastT = millis();
  }
  if(nunchuk_read()) {
    jx = nunchuk_joystickX();
    jy = nunchuk_joystickY();
    bc = nunchuk_buttonC();
    bz = nunchuk_buttonZ();
  }
  packet[0] = jx;
  packet[1] = jy;
  packet[2] = bc;
  packet[3] = bz;
  packet[4] = kp;
  packet[5] = frac_kp;
  packet[6] = ki;
  packet[7] = frac_ki;
  packet[8] = frac_ki >> 8;
  packet[9] = kd;
  packet[10] = kd >> 8;
  bts.write(packet, 11);

  unsigned long t = millis();
  while(millis() - t < 60) {
    //display loading bar and debug information
    byte loading = 0;
    if(bts.available() > 20) {
      while(bts.available()) bts.read();
    }
    for(int i = 0; bts.available() && i < 49; i++) {
      inStr[i] = (char)bts.read();
      inStr[i+1] = '\0';
      if(inStr[0] == '=') loading = 1;
    }
    if(loading) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(inStr);
      lcd.setCursor(0,1);
      lcd.print(inStr);
      delay(1400);
      return;
    } else {
      if(inStr[0] != '\0') {
        bool b = false;
        for(int i = 0; i < 3; i++) {
          if(inStr[i] == 's') b = true;
        }
        Serial.print(inStr);
        inStr[0] = '\0';
      }
    }
  }
}  
