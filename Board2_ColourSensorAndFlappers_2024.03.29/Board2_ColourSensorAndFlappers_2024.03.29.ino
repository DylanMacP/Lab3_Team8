//march 29, 2024
//by king dylan himself

#define PRINT_COLOUR                                  // uncomment to turn on output of colour sensor data
#define SERVO_PIN 41

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include <MSE2202_Lib.h>

#define MOTOR_ENABLE_SWITCH 3
#define MODE_BUTTON         0
        // GPIO0  pin 27 for Push Button 1
#define LEFT_MOTOR_A        35
        // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36
        // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37
        // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38
        // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A      15
        // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16
        // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A     11
        // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B     12
        // right encoder B signal is connected to pin 20 GPIO12 (J12)
       #define POT_R1              1
const int servoLeft = 800;     
const int servoMiddle = 1400;   
const int servoRight = 2150;  

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
const int cDisplayUpdate = 100;
        // update interval for Smart LED in milliseconds
const int cPWMRes = 8;
        // bit resolution for PWM
const int cMinPWM = 150;
        // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;
        // PWM value for maximum speed
int MinuteFortyFive =0;
int FifteenSeconds=0;


// Variables
bool heartbeatState       = true;                  // state of heartbeat LED
unsigned long lastHeartbeat  = 0;                     // time of last heartbeat state change
unsigned long curMillis      = 0;                     // current time, in milliseconds
unsigned long prevMillis     = 0;                     // start time for delay cycle, in milliseconds

const int cLeftAdjust = 6;
        // Amount to slow down left motor relative to right
const int cRightAdjust = 0;
        // Amount to slow down right motor relative to left


bool timeUp500msec = false;
        // 500 millisecond timer elapsed flag
unsigned long timerCount500msec = 0;


boolean motorsEnabled = true;
        // motors enabled flag
boolean timeUp3sec = false;
        // 3 second timer elapsed flag
boolean timeUp2sec = false;
        // 2 second timer elapsed flag
boolean timeUp200msec = false;
        // 200 millisecond timer elapsed flag
unsigned char leftDriveSpeed;
        // motor drive speed (0-255)
unsigned char rightDriveSpeed;
        // motor drive speed (0-255)
unsigned char driveIndex;
        // state index for run mode
unsigned int modePBDebounce;
        // pushbutton debounce timer count
unsigned long timerCount3sec = 0;
        // 3 second timer count in milliseconds
unsigned long timerCount2sec = 0;
        // 2 second timer count in milliseconds
unsigned long timerCount200msec = 0;
        // 200 millisecond timer count in milliseconds
unsigned long displayTime;
        // heartbeat LED update timer
unsigned long previousMicros;
        // last microsecond count
unsigned long currentMicros;
        // current microsecond count
unsigned int  robotModeIndex = 0;
unsigned char CollectionIndex;
unsigned long timerCount5sec = 0;
boolean timeUp5sec = false;

Encoders LeftEncoder = Encoders();
        // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();

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
  Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A,RIGHT_MOTOR_B); // set up motors as Drive 1
    LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B,&Bot.iLeftMotorRunning ); // set up left encoder
   RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B,&Bot.iRightMotorRunning ); // set up right encoder
    pinMode(MODE_BUTTON, INPUT_PULLUP);
       modePBDebounce = 0;
        // reset debounce timer count

        // Set up mode pushbutton
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
   long pos[] = {0, 0};
        // current motor positions
    int pot = 0;

   currentMicros = micros();
        // get current time in microseconds
   if ((currentMicros - previousMicros) >= 1000) {
        // enter when 1 ms has elapsed
      previousMicros = currentMicros;
        // record current time in microseconds
   }
      // 5 second timer, counts 5000 milliseconds
      timerCount5sec = timerCount5sec + 1;
        // increment 5 second timer count
      if (timerCount5sec > 5000) {
        // if 5 seconds have elapsed
         timerCount5sec = 0;
        // reset 5 second timer count
         timeUp5sec = true;
        // indicate that 5 seconds have elapsed
      }

      // 3 second timer, counts 3000 milliseconds
      timerCount3sec = timerCount3sec + 1;
        // increment 3 second timer count
      if (timerCount3sec > 3000) {
        // if 3 seconds have elapsed
         timerCount3sec = 0;
        // reset 3 second timer count
         timeUp3sec = true;
        // indicate that 3 seconds have elapsed
      }

      // 2 second timer, counts 2000 milliseconds
      timerCount2sec = timerCount2sec + 1;
        // increment 2 second timer count
      if (timerCount2sec > 2000) {
        // if 2 seconds have elapsed
         timerCount2sec = 0;
        // reset 2 second timer count
         timeUp2sec = true;
        // indicate that 2 seconds have elapsed
      }

      // 200 millisecond timer, counts 200 milliseconds
      timerCount200msec = timerCount200msec + 1;
        // Increment 200 millisecond timer count
      if(timerCount200msec > 200)
        // If 200 milliseconds have elapsed
        {
         timerCount200msec = 0;
        // Reset 200 millisecond timer count
         timeUp200msec = true;
        // Indicate that 200 milliseconds have elapsed
      }

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
     Bot.ToPosition("S1", servoLeft);
    } 
    timeUp500msec= false;
      }
  #endif
  }

  // Mode pushbutton debounce and toggle
      if (!digitalRead(MODE_BUTTON)) {
        // if pushbutton GPIO goes LOW (nominal push)
         // Start debounce
         if (modePBDebounce <= 25) {
        // 25 millisecond debounce time
            modePBDebounce = modePBDebounce + 1;
        // increment debounce timer count
            if (modePBDebounce > 25) {
        // if held for at least 25 mS
               modePBDebounce = 1000;
        // change debounce timer count to 1 second
            }
         }
         if (modePBDebounce >= 1000) {
        // maintain 1 second timer count until release
            modePBDebounce = 1000;
         }
      }
      else {
        // pushbutton GPIO goes HIGH (nominal release)
         if(modePBDebounce <= 26) {
        // if release occurs within debounce interval
            modePBDebounce = 0;
        // reset debounce timer count
         }
         else {
            modePBDebounce = modePBDebounce + 1;
        // increment debounce timer count
            if(modePBDebounce >= 1025) {
        // if pushbutton was released for 25 mS
               modePBDebounce = 0;
        // reset debounce timer count
               robotModeIndex++;
        // switch to next mode
               robotModeIndex = robotModeIndex & 7;
        // keep mode index between 0 and 7
               timerCount3sec = 0;
        // reset 3 second timer count
               timeUp3sec = false;
        // reset 3 second timer
            }
         }
      }

 motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH);
switch(robotModeIndex) {
         case 0: // Robot stopped
            Bot.Stop("D1");
            LeftEncoder.clearEncoder();
            RightEncoder.clearEncoder();
            driveIndex = 0;
            CollectionIndex = 0;
            // reset drive index
            timeUp2sec = false;
        // reset 2 second timer
            break;

         case 1: // 
            // Read pot to update drive motor speed
            pot = analogRead(POT_R1);
            leftDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cLeftAdjust;
            rightDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) -cRightAdjust;
    #ifdef DEBUG_ENCODER_COUNT
              if (timeUp200msec) {
                timeUp200msec = false;
      // reset 200 ms timer
                LeftEncoder.getEncoderRawCount();
      // read left encoder count
                RightEncoder.getEncoderRawCount();
      // read right encoder count
                Serial.print(F("Left Encoder count = "));
                Serial.print(LeftEncoder.lRawEncoderCount);
                Serial.print(F("  Right Encoder count = "));
                Serial.print(RightEncoder.lRawEncoderCount);
                Serial.print("\n");
              }
    #endif

            switch(CollectionIndex) {

              case 0:          
                Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); //drive ID, left speed, right speed
                //MinuteFortyFive++;
               if (LeftEncoder.lRawEncoderCount >= 5000){
                  LeftEncoder.clearEncoder();
                  RightEncoder.clearEncoder();
                  CollectionIndex ++;
                }
                break;
           
              case 1:
                Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed);
                FifteenSeconds++;
                if (FifteenSeconds >= 15000){
                  CollectionIndex ++;
                  FifteenSeconds = 0;
                }
                break;
              case 2:
                //stop robot
                Bot.Stop("D1");
                robotModeIndex = 0; //  back to 0

                break;
            }
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