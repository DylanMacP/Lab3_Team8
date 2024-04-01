// 
// MSE 2202 TCS34725 colour sensor example
// 
//  Language: Arduino (C++)
//  Target:   ESP32-S3
//  Author:   Michael Naish
//  Date:     2024 03 05 
//

#define PRINT_COLOUR                                  // uncomment to turn on output of colour sensor data
#define SERVO_PIN 41

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include <MSE2202_Lib.h>

const int servoLeft = 800;      // Adjust these values for your specific servo
const int servoMiddle = 1400;   // Middle position is typically 90 degrees
const int servoRight = 2150;   // Adjust these values for your specific servo

Motion Bot = Motion();

// Function declarations
void doHeartbeat();

// Constants
const int cHeartbeatInterval = 75;                    // heartbeat update interval, in milliseconds
const int cSmartLED          = 21;                    // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                     // number of Smart LEDs in use
const int cSDA               = 47;                    // GPIO pin for I2C data
const int cSCL               = 48;                    // GPIO pin for I2C clock
const int cTCSLED            = 14;                    // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;                    // DIP switch S1-2 controls LED on TCS32725



// Variables
bool heartbeatState       = true;                  // state of heartbeat LED
unsigned long lastHeartbeat  = 0;                     // time of last heartbeat state change
unsigned long curMillis      = 0;                     // current time, in milliseconds
unsigned long prevMillis     = 0;                     // start time for delay cycle, in milliseconds

boolean timeUp500msec = false;
        // 500 millisecond timer elapsed flag
unsigned long timerCount500msec = 0;



// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

void setup() {
  Serial.begin(115200); 

  Bot.servoBegin("S1", SERVO_PIN);

  // Set up SmartLED
  SmartLEDs.begin();                                  // initialize smart LEDs object
  SmartLEDs.clear();                                  // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0)); // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                         // set brightness [0-255]
  SmartLEDs.show();                                   // update LED

  Wire.setPins(cSDA, cSCL);                           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                  // configure GPIO to set state of TCS34725 LED 

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
}

void loop() {
   timerCount500msec = timerCount500msec + 1;
        // Increment 500 millisecond timer count
      if(timerCount500msec > 250)
        // If 500 milliseconds have elapsed
      {
         timerCount500msec = 0;
        // Reset 500 millisecond timer count
         timeUp500msec = true;
        // Indicate that 500 milliseconds have elapsed
      }

  uint16_t r, g, b, c;                                // RGBC values from TCS34725
  
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                      // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
#ifdef PRINT_COLOUR            
      //Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
      if(timeUp500msec == true){
    if((r >= 22 && r <= 27) && (g >= 31 && g <= 41) && (b >=22 && b <= 26)){
      Serial.printf("That's a Little Green Guy R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
      Bot.ToPosition("S1", servoLeft);
    } else if((r >= 28 && r <= 36) && (g >= 30 && g <= 41) && (b >=27 && b <= 36)) {
        Serial.printf("It's Blank u dumb motherfucker R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
        Bot.ToPosition("S1", servoMiddle);
    }
      else{
     Serial.printf("That's Not a Little Green Guy R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
     Bot.ToPosition("S1", servoRight);
    } 
    timeUp500msec= false;
      }
  #endif
}
  doHeartbeat();                                      // update heartbeat LED
}


// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                               // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat time for the next update
    LEDBrightnessIndex++;                             // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) { // if all defined levels have been used
      LEDBrightnessIndex = 0;                         // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]); // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 250, 0)); // set pixel colours to green
    SmartLEDs.show();                                 // update LED
  }
}