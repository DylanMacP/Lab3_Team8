#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"


  /* 
   Lab 003 Team 8 Drive System Code For MSEDuino - 2:
   Robot: Mark
   This Code Controls:
   2 Encoders for the Comvines
   1 Servo Motor
   1 Colour Sensor
   This code was once again taken from Lab 4 as a baseline so some components may be uneccesary but they were left
   in in-case they were needed later.
   */


#define PRINT_COLOUR

#define SERVO_PIN 41

#define LEFT_MOTOR_A 35
// GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B 36
// GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A 37
// GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B 38
// GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A 15
// left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B 16
// left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A 11
// right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B 12
// right encoder B signal is connected to pin 20 GPIO12 (J12)
#define MODE_BUTTON 0
// GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3
// DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1 1
// when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1

unsigned char leftDriveSpeed;
// motor drive speed (0-255)
unsigned char rightDriveSpeed;
// motor drive speed (0-255)
unsigned char driveIndex;
// state index for run mode
unsigned char CollectionIndex;
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

unsigned int modePBDebounce;
// pushbutton debounce timer count
unsigned int robotModeIndex = 0;
int TimerCountRobotTest = 0;
int Tracker = 0;

const int cHeartbeatInterval = 75;  // heartbeat update interval, in milliseconds
const int cSmartLED = 21;           // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount = 1;       // number of Smart LEDs in use
const int cSDA = 47;                // GPIO pin for I2C data
const int cSCL = 48;                // GPIO pin for I2C clock
const int cTCSLED = 14;             // GPIO pin for LED on TCS34725
const int cLEDSwitch = 46;          // DIP switch S1-2 controls LED on TCS32725
const int servoLeft = 800;     
const int servoMiddle = 1400;   
const int servoRight = 2150;  
const int cPWMRes = 8;
        // bit resolution for PWM
const int cMinPWM = 150;
        // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;
        // PWM value for maximum speed 

Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

Motion Bot = Motion();

Encoders LeftEncoder = Encoders();
// Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();
// Instance of Encoders for right encoder data

bool heartbeatState = true;       // state of heartbeat LED
unsigned long lastHeartbeat = 0;  // time of last heartbeat state change
unsigned long curMillis = 0;      // current time, in milliseconds
unsigned long prevMillis = 0;

// Added Variables For Encoder Timers & Colour Sensor
bool tcsFlag = 0;
int pot = 0;
int FifteenSeconds=0;
int MinuteFortyFive=0;

//TIMER FOR COLOUR SENSOR
bool timeUp250mSec = false;
unsigned long timerCount250mSec = 0;

//TIMER For Driving
bool timeUp2sec = false;
unsigned long timerCount2sec = 0;
unsigned long timerCount3sec = 0;
bool timeUp3sec = false;


void setup() {
  Serial.begin(115200);
  Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B);  // set up motors as Drive 1
  LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning);       // set up left encoder
  RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning);   // set up right encoder
  Bot.servoBegin("S1", SERVO_PIN);

  SmartLEDs.begin();                                     // initialize smart LEDs object
  SmartLEDs.clear();                                     // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 0, 0));  // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                            // set brightness [0-255]
  SmartLEDs.show();


  Wire.setPins(cSDA, cSCL);  // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);  // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);

  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
  pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);
  // set up motor enable switch with internal pullup
  pinMode(MODE_BUTTON, INPUT_PULLUP);
  // Set up mode pushbutton
  modePBDebounce = 0;
  // reset debounce timer count
}

void loop() {
  // COLOUR SENSOR SECTION
  timerCount250mSec = timerCount250mSec + 1;
  // Increment 100 millisecond timer count
  if (timerCount250mSec > 250)
  // If 100 milliseconds have elapsed
  {
    timerCount250mSec = 0;
    // Reset 100 millisecond timer count
    timeUp250mSec = true;
    // Indicate that 100 milliseconds have elapsed
  }
  uint16_t r, g, b, c;  // RGBC values from TCS34725

  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));  // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                    // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                 // get raw RGBC values
#ifdef PRINT_COLOUR
      //Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
    if (timeUp250mSec == true) {
      if ((r >= 17 && r <= 24) && (g >= 24 && g <=31) && (b >= 17 && b <= 23) && (c >= 62 && c <= 75)) {// GREEN VALUES FOR COLOUR SENSOR, NEED ADJUSTMENT EACH TIME
        Serial.printf("That's a Little Green Guy R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
        Bot.ToPosition("S1", servoLeft); //Left Position for the servo moves marbles to the right
        Tracker = 1; 
      } else if ((r >= 25 && r <= 32) && (g >= 27 && g <= 35) && (b >= 24 && b <= 30) && (c>=77)) { // BLANK Values for Colour Sensor
        Serial.printf("It's Blank R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
        Bot.ToPosition("S1", servoMiddle); // Middle Position holds the marbles in place until their colours are read
      } else {
        Serial.printf("That's Not a Little Green Guy R: %d, G: %d, B: %d, C %d\n", r, g, b, c); // Default value for anything other than blank or green, send these to the bad side
        Bot.ToPosition("S1", servoRight);
        Tracker = 1;
      }
      timeUp250mSec = false;
    }
#endif
  }
  if (Tracker == 1) { // On Both the left and right position the tracker is put to 1 so that the servo goes back into place (middle) after 50
    TimerCountRobotTest++;
    if(TimerCountRobotTest >= 50){
    Bot.ToPosition("S1", servoMiddle);
    Tracker = 0;
    TimerCountRobotTest = 0;
    }
  }
  //END OF COLOUR SENSOR SECTION

  //FLAPPER SECTION
  //Assorted Timers
        timerCount3sec = timerCount3sec + 1;
        // increment 3 second timer count
      if (timerCount3sec > 3000) {
        // if 3 seconds have elapsed
         timerCount3sec = 0;
        // reset 3 second timer count
         timeUp3sec = true;
        // indicate that 3 seconds have elapsed
      }
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
    long pos[] = {0, 0};
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
  switch (robotModeIndex) {
    case 0:  // Robot stopped
      Bot.Stop("D1");
      LeftEncoder.clearEncoder();
      RightEncoder.clearEncoder();
      driveIndex = 0;
      CollectionIndex = 0;
      // reset drive index
      timeUp2sec = false;
      // reset 2 second timer
      break;

    case 1:  //
      // Read pot to update drive motor speed
      pot = analogRead(POT_R1);
      leftDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);
      rightDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);
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

      switch (CollectionIndex) {

        //Run the large combine for the duration of the run
        case 0:
          Bot.Reverse("D1", leftDriveSpeed, 0);  //drive ID, left speed, right speed
         MinuteFortyFive++;
          if (MinuteFortyFive >= 12000) { //This is closer to 45 seconds to match our run duration
            CollectionIndex++;
            MinuteFortyFive =0;
          }
          break;

        /*Following the ~45 seconds of spinning the bot should have made it back to base station, at which point the smaller combine 
        triggers which is responsible for depositing the green marbles into the basin */
        case 1:
          Bot.Forward("D1", 0, rightDriveSpeed);
          FifteenSeconds++;
          if (FifteenSeconds >= 3000) {
            CollectionIndex++;
            FifteenSeconds = 0;
          }
          break;
          //Following the deposit the bot stops
        case 2:
          //stop robot
          Bot.Stop("D1");
          robotModeIndex = 0;  //  back to 0

          break;
      }
  }
  doHeartbeat();  // update heartbeat LED
}

// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();  // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                               // update the heartbeat time for the next update
    LEDBrightnessIndex++;                                    // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {  // if all defined levels have been used
      LEDBrightnessIndex = 0;                                // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);  // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 250, 0));            // set pixel colours to green
    SmartLEDs.show();                                                  // update LED
  }
}

