#define DEBUG_DRIVE_SPEED    1
#define DEBUG_ENCODER_COUNT  1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>

// Function declarations
void Indicator();
        // for mode/heartbeat on Smart LED

// Port pin constants
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
#define MODE_BUTTON         0
        // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3
        // DIP Switch S1-1 pulls Digital pin D3 to ground when on,
connected to pin 15 GPIO3 (J3)
#define POT_R1              1
        // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is
connected to Poteniometer R1
#define SMART_LED           21
        // when DIP Switch S1-4 is on, Smart LED is connected to pin
23 GPIO21 (J21)
#define SMART_LED_COUNT     1
        // number of SMART LEDs in use
#define IR_DETECTOR         14
        // GPIO14 pin 17 (J14) IR detector input
#define SHOULDER_SERVO      41
        // GPIO41 pin 34 (J41) Servo 1
#define CLAW_SERVO          42
        // GPIO42 pin 35 (J42) Servo 2

// Constants
const int cDisplayUpdate = 100;
        // update interval for Smart LED in milliseconds
const int cPWMRes = 8;
        // bit resolution for PWM
const int cMinPWM = 150;
        // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;
        // PWM value for maximum speed

//=====================================================================================================================
//
// IMPORTANT: The constants in this section need to be set to
appropriate values for your robot.
//            You will have to experiment to determine appropriate values.

const int cClawServoOpen = 1100;
        // Value for open position of claw
const int cClawServoClosed = 2050;
        // Value for closed position of claw
const int cArmServoUp = 2200;
        // Value for shoulder of arm fully up
const int cArmServoDown = 1300;
        // Value for shoulder of arm fully down
const int cLeftAdjust = 0;
        // Amount to slow down left motor relative to right
const int cRightAdjust = 0;
        // Amount to slow down right motor relative to left

//
//=====================================================================================================================

// Variables
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
unsigned int potClawSetpoint;
        // desired position of claw servo read from pot
unsigned int potShoulderSetpoint;
        // desired position of shoulder servo read from pot
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

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

// smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0;
unsigned char LEDBrightnessLevels[] =
{5,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,

240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15};

unsigned int  robotModeIndex = 0;
        // robot operational state
unsigned int  modeIndicator[6] = {
        // colours for different modes
   SmartLEDs.Color(255,0,0),
        //   red - stop
   SmartLEDs.Color(0,255,0),
        //   green - run
   SmartLEDs.Color(0,0,255),
        //   blue - empty case
   SmartLEDs.Color(255,255,0),
        //   yellow - empty case
   SmartLEDs.Color(0,255,255),
        //   cyan - empty case
   SmartLEDs.Color(255,0,255)
        //   magenta - empty case
};

// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();
        // Instance of Motion for motor control
Encoders LeftEncoder = Encoders();
        // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();
        // Instance of Encoders for right encoder data
IR Scan = IR();
        // instance of IR for detecting IR signals

//my code
unsigned char pingPongFinder;
        // state index for ping pong ball finding mode
unsigned long timerCount5sec = 0;
        // 5 second timer count in milliseconds
boolean timeUp5sec = false;
        // 5 second timer elapsed flag
int turned_amount;
        // to record how much it turned when looking for the ball
boolean timeUp500msec = false;
        // 500 millisecond timer elapsed flag
unsigned long timerCount500msec = 0;
        // 500 millisecond timer count in milliseconds
int iteration = 0;
        // for smoothing arm raise

void setup() {
#if defined DEBUG_DRIVE_SPEED || DEBUG_ENCODER_COUNT
   Serial.begin(115200);
#endif

  // Set up motors and encoders
   Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A,
RIGHT_MOTOR_B); // set up motors as Drive 1
   Bot.servoBegin("S1", CLAW_SERVO);
        // set up claw servo
   Bot.servoBegin("S2", SHOULDER_SERVO);
        // set up shoulder servo
   LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B,
&Bot.iLeftMotorRunning ); // set up left encoder
   RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B,
&Bot.iRightMotorRunning ); // set up right encoder

   Scan.Begin(IR_DETECTOR, 1200);
        //set up IR Detection @ 1200 baud

   // Set up SmartLED
   SmartLEDs.begin();
        // initialize smart LEDs object (REQUIRED)
   SmartLEDs.clear();
        // clear pixel
   SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));
        // set pixel colors to 'off'
   SmartLEDs.show();
        // send the updated pixel colors to the hardware

   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);
        // set up motor enable switch with internal pullup
   pinMode(MODE_BUTTON, INPUT_PULLUP);
        // Set up mode pushbutton
   modePBDebounce = 0;
        // reset debounce timer count
}

void loop() {
   long pos[] = {0, 0};
        // current motor positions
   int pot = 0;
  //  LeftEncoder.getEncoderRawCount();                            //
read left encoder count
  //   RightEncoder.getEncoderRawCount();
   Serial.print("Left Encoder count = ");
   Serial.print(LeftEncoder.lRawEncoderCount);
   Serial.print(", Right Encoder count = ");
   Serial.println(RightEncoder.lRawEncoderCount);
                                        // raw ADC value from pot

   currentMicros = micros();
        // get current time in microseconds
   if ((currentMicros - previousMicros) >= 1000) {
        // enter when 1 ms has elapsed
      previousMicros = currentMicros;
        // record current time in microseconds

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

      // 500 millisecond timer, counts 500 milliseconds
      timerCount500msec = timerCount500msec + 1;
        // Increment 500 millisecond timer count
      if(timerCount500msec > 500)
        // If 500 milliseconds have elapsed
      {
         timerCount500msec = 0;
        // Reset 500 millisecond timer count
         timeUp500msec = true;
        // Indicate that 500 milliseconds have elapsed
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

      // check if drive motors should be powered
      motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH);
        // if SW1-1 is on (low signal), then motors are enabled

      // modes
      // 0 = Default after power up/reset. Robot is stopped.
      // 1 = Press mode button once to enter.        Run robot
      // 2 = Press mode button twice to enter.       Test IR receiver
      // 3 = Press mode button three times to enter. Test claw servo
      // 4 = Press mode button four times to enter.  Test shoulder servo
      // 5 = Press mode button five times to enter.  Add your code to
do something
      // 6 = Press mode button six times to enter.   Add your code to
do something
      switch(robotModeIndex) {
         case 0: // Robot stopped
            Bot.Stop("D1");
            // LeftEncoder.clearEncoder();
           // clear encoder counts
            // RightEncoder.clearEncoder();
            driveIndex = 0;
            pingPongFinder = 0;
            // reset drive index
            timeUp2sec = false;
        // reset 2 second timer
            break;

         case 1: // case for finding ping pong ball
            // Read pot to update drive motor speed
            pot = analogRead(POT_R1);
            leftDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cLeftAdjust;
            rightDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) -
cRightAdjust;
            //LeftEncoder.getEncoderRawCount();
    // read left encoder count
            //RightEncoder.getEncoderRawCount();
    // read right encoder count

#ifdef DEBUG_DRIVE_SPEED
              Serial.print(F(" Left Drive Speed: Pot R1 = "));
              Serial.print(pot);
              Serial.print(F(", mapped = "));
              Serial.println(leftDriveSpeed);
#endif
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
            switch(pingPongFinder) {

              //need to first go forward across 1 and a half sheets of paper
              case 0:

                //start with arm up
                Bot.ToPosition("S2", cArmServoUp);
    // update shoulder servo position

                Serial.print("case 0 start - drive forward");

                //move forward
                Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); //
drive ID, left speed, right speed

                //stop when 50 cm is reached
                if (LeftEncoder.lRawEncoderCount >= 3000){
                  Serial.print("case 0 end");
                  pingPongFinder = 1;
                }
                break;

              // need to rotate 90 degrees CCW
              case 1:
                Serial.print("case 1 start - turn 90 CCW");

                //rotate to the left (CCW)
                Bot.Left("D1", leftDriveSpeed, rightDriveSpeed);
                //Bot.Right("D1", leftDriveSpeed, rightDriveSpeed);
                // turn until we are at 90 degrees
                if (LeftEncoder.lRawEncoderCount <= 2700){
                  Serial.print("case 1 end");
                  pingPongFinder = 2;
                }

                break;

              // now we need to go to the end of the next piece of paper
              case 2:

                Serial.print("case 2 start - drive forward again");

                // move forward
                Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); //
drive ID, left speed, right speed

                // move until we are at the end of the paper
                if (LeftEncoder.lRawEncoderCount >= 4600){
                  Serial.print("case 2 end");
                  turned_amount = LeftEncoder.lRawEncoderCount;
//store exact encoder count
                  pingPongFinder = 3;
                  Bot.Left("D1", 158, 158); //initially turn left
                }
                break;

              //stop motion and find the ball
              case 3:
                Serial.print("case 3 start - find ball");

                //open claw for next step
                Bot.ToPosition("S1", cClawServoOpen);
      // update claw servo position
                Bot.ToPosition("S2", cArmServoDown);
      // update shoulder servo position

                // scan for IR light - check if IR is returning "U"
                if(Scan.Available()){
                  if (Scan.Get_IR_Data() == 'U'){
       // when first motor encoder value is less than or equal to 2739
pulses
                    Bot.Stop("D1");
                    Serial.print("case 3 - ball found");
                    turned_amount = LeftEncoder.lRawEncoderCount;
                    Serial.print(turned_amount);
                    timeUp2sec = false;
       // reset 2 second timer
                    timeUp500msec = false;
       // reset 500ms second timer
                    pingPongFinder = 4;
                    break;
                  }
                }

                //if its too far left, go back right
                if (LeftEncoder.lRawEncoderCount <= turned_amount - 85){
                  Serial.print("low encode - switching to right");
                  Bot.Right("D1", 158, 158);
                }

                //if its too far right, go left
                if (LeftEncoder.lRawEncoderCount >= turned_amount + 85){
                  Serial.print("high encode - switching to left");
                  Bot.Left("D1", 158, 158);
                }

                break;

              // move the arm up to ball height
              // moving the arm too quickly made my robot move a bit,
and then the claw would be innaccurate
              // this slow moving fix allows the arm to not influence
which way the robot is facing
              case 4:
                if (timeUp2sec) {
     // update drive state after 2 seconds

                  Serial.print("case 4 start - slowly move servo arm");

                  // i want to slowly move the arm up to avoid
throwing off the accuracy
                  if (timeUp500msec){
                    Bot.ToPosition("S2",
(cArmServoDown+(iteration*25)));                         // update
shoulder servo position to the ball height
                    iteration++;
                           // add to the iteration counter
                    timeUp500msec = false;
                           // reset 500ms second timer
                  }

                  // did 16 iterations of 25 each to equal 400, so the
total value is 1600 (400 + initial 1200)
                  if (iteration > 16){
                      pingPongFinder = 5;
                  }
                }

                  break;

              //move to the ball, slowly
              case 5:

                if (timeUp2sec) {
     // update after 2 seconds
                  //move forward to the ball
                  Serial.print("case 5 start - move to ball");

                  Bot.Forward("D1", 165, 165);               // move forward

                  // drive to until we reach the end of the paper
                  Serial.print("need to get to, at");
                  Serial.print(turned_amount+850);
                  Serial.print(LeftEncoder.lRawEncoderCount);

                  if (LeftEncoder.lRawEncoderCount >= turned_amount+850){
                    timeUp5sec = false;
         // reset 5 second timer
                    Serial.print("case 5 end");
                    Bot.Stop("D1"); // drive ID, left speed, right speed
                    timeUp500msec = false;
          // reset 500 ms timer
                    pingPongFinder = 6;
                  }
                }

                break;

              //grab the ball
              case 6:
                if (timeUp500msec) {
        // update drive state after 500ms
                  Serial.print("case 6 start - grab ball");

                  //close the claw
                  Bot.ToPosition("S1", cClawServoClosed);

                  if (timeUp5sec) {
       // update after 5 seconds, used this to check if the ball was
grabbed nicely

                    // finish with the arm straight up to hold the ball better
                    Bot.ToPosition("S2", cArmServoUp);
        // update shoulder servo position
                    pingPongFinder = 7;
                  }
                }

                break;

              //go backwards 2 sheets
              case 7:

                Serial.print("case 7 start - reverse back first part");

                Bot.Reverse("D1", leftDriveSpeed, rightDriveSpeed); //
drive ID, left speed, right speed

                // go back the same
                if (LeftEncoder.lRawEncoderCount <= 2700){
                  Serial.print("case 7 end");
                  pingPongFinder = 8;
                }

                break;

              //turn CW
              case 8:

                Serial.print("case 8 start - turn 45 CW");

                Bot.Right("D1", leftDriveSpeed, rightDriveSpeed);   //
drive ID, left speed, right speed

                // turn a bit
                if (LeftEncoder.lRawEncoderCount >= 2800){
                  Serial.print("case 8 end");
                  pingPongFinder = 9;
                }

                break;

              // reverse just a bit, makes it easier to get around the turn
              case 9:

                Serial.print("case 9 start - reverse back second part");

                Bot.Reverse("D1", leftDriveSpeed, rightDriveSpeed); //
drive ID, left speed, right speed

                // go back a bit
                if (LeftEncoder.lRawEncoderCount <= 2600){
                  Serial.print("case 9 end");
                  pingPongFinder = 10;
                }

                break;

              // turn more CW
              case 10:

                Serial.print("case 10 start - turn a bit more");

                // turn CW
                Bot.Right("D1", leftDriveSpeed, rightDriveSpeed);   //
drive ID, left speed, right speed

                // turn more
                if (LeftEncoder.lRawEncoderCount >= 2800){
                  Serial.print("case 10 end");
                  pingPongFinder = 11;
                }

                break;

              // reverse back to the start
              case 11:

                Serial.print("case 11 start - reverse back to the start");

                Bot.Reverse("D1", leftDriveSpeed, rightDriveSpeed); //
drive ID, left speed, right speed

                // go back the same
                if (LeftEncoder.lRawEncoderCount <= 0){
                  // when first motor encoder value is greater than or
equal to 4110 pulses
                  Serial.print("case 11 end");
                  pingPongFinder = 12;
                }

                break;

              //drop the ball and turn off
              case 12:
                //stop robot
                Bot.Stop("D1");
                Bot.ToPosition("S1", cClawServoOpen);
           // update claw servo position

                robotModeIndex = 0; //  back to 0

                break;

            }
            break;

         case 2:  //add your code to do something
            robotModeIndex = 0; //  !!!!!!!  remove if using the case
            break;

         case 3:  //add your code to do something
            robotModeIndex = 0; //  !!!!!!!  remove if using the case
            break;

         case 4:  //add your code to do something
            robotModeIndex = 0; //  !!!!!!!  remove if using the case
            break;

         case 5: //add your code to do something
            robotModeIndex = 0; //  !!!!!!!  remove if using the case
            break;

         case 6: //add your code to do something
            robotModeIndex = 0; //  !!!!!!!  remove if using the case
            break;

      }

      // Update brightness of heartbeat display on SmartLED
      displayTime++;
       // count milliseconds
      if (displayTime > cDisplayUpdate) {
       // when display update period has passed
         displayTime = 0;
       // reset display counter
         LEDBrightnessIndex++;
       // shift to next brightness level
         if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {
       // if all defined levels have been used
            LEDBrightnessIndex = 0;
       // reset to starting brightness
         }
         SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);
   // set brightness of heartbeat LED
         Indicator();
       // update LED
      }
   }
}

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator() {
  SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);
       // set pixel colors to = mode
  SmartLEDs.show();
       // send the updated pixel colors to the hardware
}