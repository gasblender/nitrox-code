/*****************************************************************************
 *
 * (c) 2019-2020 Éric Seigne <eric@gasblender.org>
 *
 * gasblender o2 analyser - gasblender.org
 * source code derivated from ej's o2 oled analyzer - v0.21 (http://ejlabs.net/arduino-oled-nitrox-analyzer)
 *
 * License
 * -------
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *         You should have received a copy of the GNU General Public License
 *         along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>             // Arduino SPI library
#include <Adafruit_ADS1X15.h>
#include <EEPROM.h>
#include "RunningAverage.h"
#include "pitches.h"

#define RA_SIZE 5
RunningAverage RA(RA_SIZE);

Adafruit_ADS1115 ads;

#define TFT_CS     10
#define TFT_RST    8  // define reset pin, or set to -1 and connect to Arduino RESET pin
#define TFT_DC     9  // define data/command pin
// #define SDA        24
// #define SCL        23

// Initialize Adafruit ST7789 TFT library
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

double calibrationv;
float multiplier;
byte previous = HIGH;
unsigned long firstTime; // how long since the button was first pressed
int active = 0;
double result_max = 0;
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

// not used for gasblender 1.0 version
//const int buttonPin = 2; // push button
const int buzzerPin = 2; // buzzer (D2)
//const int ledPin = 13; // led
//const int cal_holdTime = 2; // 2 sec button hold to calibration
//const int mod_holdTime = 3; // 3 sec hold to po2 mod change
//const int max_holdtime = 4; // 4 sec hold to reset max o2 result
//long millis_held;    // How long the button was held (milliseconds)
//long secs_held;      // How long the button was held (seconds)
//long prev_secs_held; // How long the button was held in the previous check

/*
  Calculate MOD (Maximum Operating Depth)
*/
float max_po1 = 1.30;
const float max_po2 = 1.60;
float cal_mod (float percentage, float ppo2 = 1.4) {
  return 10 * ( (ppo2 / (percentage / 100)) - 1 );
}

// make tada for x time
void tada(int x = 1) { 
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(buzzerPin, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // The note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(buzzerPin);
  }
}

void beep(int x = 1) { 
  int noteDuration = 500;
  tone(buzzerPin, melody[x], noteDuration);
  delay(400);
  noTone(buzzerPin);
}

void read_sensor(int adc = 0) {
  int16_t millivolts = 0;
  Serial.println(F("read_sensor ..."));
  millivolts = ads.readADC_Differential_0_1();

  /* debug  erics */
    Serial.print("read_sensor o2: ");
    Serial.print(millivolts);
    Serial.print("\n");
  /* */
  //On evite de stocker des valeurs erronnees
  if (millivolts > 0) {
    RA.addValue(millivolts);
  }

/*

 millivolts = 0;
  millivolts = ads.readADC_Differential_2_3();
*/
  /* debug  erics */
    /*Serial.print("read_sensor He: ");
    Serial.print(millivolts);
    Serial.print("\n");
*/

  delay(200);
}

void setup(void) {
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(9600);
  Serial.print("Lancement de l'analyseur ...\n");

  // if the display has CS pin try with SPI_MODE0
  display.init(240, 240, SPI_MODE2);    // Init ST7789 display 240x240 pixel
  display.setRotation(1); //rotate screen if needed
  //display.setSPISpeed(0);
  // if the screen is flipped, remove this command
  //display.setRotation(2);
  Serial.println(F("Initialized"));
  uint16_t time = millis();
  display.fillScreen(ST77XX_BLACK);
  time = millis() - time;
  delay(500);

  /* power saving stuff for battery power */
  // Disable ADC
  // ADCSRA = 0;
  // Disable the analog comparator by setting the ACD bit
  // (bit 7) of the ACSR register to one.
  // ACSR = B10000000;
  // Disable digital input buffers on all analog input pins
  // DIDR0 = DIDR0 | B00111111;

  Serial.println(F("\nI2C PINS"));
  Serial.print(F("\tSDA = ")); Serial.println(SDA);
  Serial.print(F("\tSCL = ")); Serial.println(SCL);

  Serial.println(F("Before ADS init"));
  ads.setGain(GAIN_TWO);
  multiplier = 0.0625F;
  ads.begin(0x48); // ads1115 start
  Serial.println(F("ADS initialized"));


  int16_t val_0 = ads.readADC_SingleEnded(0);
  // int16_t val_1 = ads.readADC_SingleEnded(1);
  // int16_t val_2 = ads.readADC_SingleEnded(2);
  // int16_t val_3 = ads.readADC_SingleEnded(3);

  Serial.print("\tAnalog0: "); Serial.print(val_0); Serial.print('\t');
  // Serial.print("\tAnalog1: "); Serial.print(val_1); Serial.print('\t');
  // Serial.print("\tAnalog2: "); Serial.print(val_2); Serial.print('\t');
  // Serial.print("\tAnalog3: "); Serial.print(val_3); Serial.print('\t');
  Serial.println();

//  pinMode(buttonPin, INPUT_PULLUP);
  Serial.println(F("SplashScreen started"));
  display.fillScreen(ST77XX_BLACK);
  display.setCursor(0, 30);
  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(4);
  display.println(F(""));
  display.println(F("gasblender"));
  display.setTextSize(2);
  display.setTextColor(ST77XX_YELLOW);
  display.println(F("   Open-Source"));
  display.setTextColor(ST77XX_ORANGE);
  display.println(F("  Open-Hardware"));
  display.setTextColor(ST77XX_RED);
  display.println(F(" Nitrox Analyser"));
  display.setTextColor(ST77XX_WHITE);
  display.println(F(""));
  display.println(F("  gasblender.org"));
  display.println(F("        version 1.2"));
  delay(1500);
  Serial.println(F("SplashScreen end"));

  RA.clear();
  for (int cx = 0; cx <= RA_SIZE; cx++) {
    read_sensor(0);
  }

  calibrationv = EEPROMReadInt(0);
  //calibration forced on boot
  //if (calibrationv < 100) {
  calibrationv = calibrate(0);
  //}

  Serial.print("Fin de la phase de lancement...\n");
  display.fillScreen(ST77XX_BLACK);
  beep(1);
}

void EEPROMWriteInt(int p_address, int p_value)
{
  /*
    Serial.print("EEPROMWriteInt ");
    Serial.print(p_address);
    Serial.print(" : ");
    Serial.print(p_value);
    Serial.print("\n");
  */
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

unsigned int EEPROMReadInt(int p_address)
{
  Serial.println(F("EEPROMReadInt"));
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  Serial.println(F("EEPROMReadInt end"));
  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

int calibrate(int x) {
  /* */
    Serial.print("calibrate:");
    Serial.print(x);
    Serial.print("\n");
  /* */
  //delay(5000);

  display.fillScreen(ST77XX_BLACK);
  display.setTextColor(ST77XX_WHITE);
  display.setCursor(0, 0);
  display.setTextSize(4);
  display.println(F("Calibrate"));
  display.println(" ");
  display.setTextSize(2);

  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(4);
  display.println(F(""));
  display.println(F("gasblender"));
  display.setTextSize(2);
  display.setTextColor(ST77XX_YELLOW);
  display.println(F("   Open-Source"));
  display.setTextColor(ST77XX_ORANGE);
  display.println(F("  Open-Hardware"));
  display.setTextColor(ST77XX_RED);
  display.println(F(" Nitrox Analyser"));
  display.setTextColor(ST77XX_WHITE);
  display.println(F(""));
  display.println(F("  gasblender.org"));
  display.println(F("        version 1.2"));


  //RA.clear();
  double result;
  for (int cx = 0; cx <= RA_SIZE; cx++) {
    if(cx == 18) {
      display.println(".");
      display.print(".");
    }
    else {
      display.print(".");
    }
    read_sensor(0);
  }
  result = RA.getAverage();
  result = abs(result);
  EEPROMWriteInt(x, result); // write to eeprom

  delay(1000);
  active = 0;
  Serial.println(result);
  return result;
}

void analysing(int x, int cal) {
  double currentmv = 0;
  double result;
  double mv = 0.0;

  read_sensor(0);
  currentmv = RA.getAverage();
  currentmv = abs(currentmv);

  result = (currentmv / cal) * 20.9;
  if (result > 99.9) result = 99.9;
  mv = currentmv * multiplier;

  display.fillScreen(ST77XX_BLACK);
  display.setTextColor(ST77XX_WHITE);
  display.setCursor(0, 0);

  if (mv < 0.02 || result <= 0) {
    display.setTextSize(4);
    display.println(F("Sensor"));
    display.print(F("Error!"));
  } else {
    display.setTextSize(8);
    display.print(result,1);
    display.println(F("%"));

    if (result >= result_max) {
      result_max = result;
    }

    display.setTextSize(3);
    display.setCursor(0, 70);
    display.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
    display.print(F(" Max "));
    display.print(result_max, 1);
    display.println(F("%   "));
    //display.setCursor(75,31);
    display.print(F(" cell. "));
    display.print(mv, 2);
    display.println(F("mv"));

    display.setCursor(230, 70);
    display.setTextColor(ST77XX_WHITE);
    if (active % 4) {
      display.print(F("."));
      //pour eviter un overflow du int ...
    }
    else {
      display.print(F("o"));
    }

    display.setTextColor(ST77XX_WHITE);
    display.setCursor(0, 130);
    display.println(F("Calcul PMAX:"));
    // display.println(F("ppO2"));

    
    display.setCursor(0, 160);
    display.setTextSize(4);
    display.setTextColor(ST77XX_GREEN);
    display.print(F(" "));
    display.print(max_po1, 1);
    display.setTextColor(ST77XX_WHITE);
    display.print(F("| "));
    display.setTextColor(ST77XX_ORANGE);
    display.print(max_po2, 1);
    display.setTextColor(ST77XX_WHITE);
    display.setCursor(0, 200);
    display.setTextColor(ST77XX_GREEN);
    display.print(cal_mod(result, max_po1), 1);
    display.setTextColor(ST77XX_WHITE);
    display.print(F("|"));
    display.setTextColor(ST77XX_ORANGE);
    display.print(cal_mod(result, max_po2), 1);
    display.println(F("m"));
    // menu - not used for gasblender 1.0
    /*
    if (secs_held < 5 && active > 16) {
      display.setTextSize(2);
      display.setCursor(0, 31);
      display.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
      if (secs_held >= cal_holdTime && secs_held < mod_holdTime) {
        display.print(F("   CAL    "));
      }
      if (secs_held >= mod_holdTime && secs_held < max_holdtime) {
        display.print(F("   PO2    "));
      }
      if (secs_held >= max_holdtime && secs_held < 10) {
        display.print(F("   MAX    "));
      }
    }
    */
  }
}

/* not used for gasblender 1.0
void lock_screen(long pause = 5000) {
  beep(1);
  display.setTextSize(1);
  display.setCursor(0, 31);
  display.setTextColor(0xFFFF, 0);
  display.print(F("                "));
  display.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
  display.setCursor(0, 31);
  display.print(F("======= LOCK ======="));
  for (int i = 0; i < pause; ++i) {
    while (digitalRead(buttonPin) == HIGH) {
    }
  }
  active = 0;
}
*/
/*
void po2_change() {
  if (max_po1 == 1.3) max_po1 = 1.4;
  else if (max_po1 == 1.4) max_po1 = 1.5;
  else if (max_po1 == 1.5) max_po1 = 1.3;

  display.clearDisplay();
  display.setTextColor(ST77XX_WHITE);
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println(F("pO2 set"));
  display.print(max_po1);
  beep(1);
  delay(1000);
  active = 0;
}
*/
/*
  void max_clear() {
  result_max = 0;
  display.clearDisplay();
  display.setTextColor(ST77XX_WHITE);
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println(F("Max result"));
  display.print(F("cleared"));
  beep(1);
  delay(1000);
  active = 0;
}
*/
void loop(void) {
/*

 int current = digitalRead(buttonPin);

  if (current == LOW && previous == HIGH && (millis() - firstTime) > 200) {
    firstTime = millis();
    active = 17;
  }

  millis_held = (millis() - firstTime);
  secs_held = millis_held / 1000;

  if (millis_held > 2) {
    if (current == HIGH && previous == LOW) {
      if (secs_held <= 0) {
        lock_screen();
      }
      if (secs_held >= cal_holdTime && secs_held < mod_holdTime) {
        calibrationv = calibrate(0);
      }
      if (secs_held >= mod_holdTime && secs_held < max_holdtime) {
        po2_change();
      }
      if (secs_held >= max_holdtime && secs_held < 10) {
        max_clear();
      }
    }
  }

  previous = current;
  prev_secs_held = secs_held;
*/
  analysing(0, calibrationv);
  delay(3000);

  active++;
}
