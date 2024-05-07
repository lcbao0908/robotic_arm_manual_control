#include "RoboClaw.h"
#include "SoftwareSerial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

// MOTOR SETUP
// ------------------- ** SWITCH SETUP ** --------------------
int switch2nd;
int switch3rd;
#define LIMIT_SWITCH_2ND 10
#define LIMIT_SWITCH_3RD 11

// ------------------- ** 1st Axis ODESC and NEO Motor ** ---------------------
byte sparkMaxPin = 33;
Servo sparkmax;

// ------------------- ** 2nd Axis & ROBOCLAW ** ---------------------
RoboClaw roboclaw2nd = RoboClaw(&Serial2, 10000);
#define ROBOCLAW_ADDRESS_2ND 0x80

// ------------------- ** 3rd Axis Stepper Servo Motor ** ---------------------
#define PUL 37 // Black Cable of 3rd Axis PUL+, PIN MUST BE PWM PIN ON TEENSY PINOUT. Pin 37 on Teensy 4.1
#define DIR 39 // Red Cable of 3rd Axis DIR+. Pin 39 on Teensy 4.1
// Orange and Brown is PUL- and DIR-, connecting to Ground
double speedPercent; // Speed control for the slider

// ------------------- ** 4th and 5th Axis GoBILDA & ROBOCLAW ** ---------------------
RoboClaw roboclaw = RoboClaw(&Serial1, 10000);
#define ROBOCLAW_ADDRESS_4TH_5TH 0x81

// ------------------- ** Gripper ** ---------------------
byte servoPin = 6;
Servo servo;
int val = 1600;
int pos = 0;
int new_val;
float t = 0;

// ------------------- ** Kinematic Global Variables ** --------------------- 
void setup() {
  Serial.begin(115200);
  roboclawSetup(); // Roboclaw Setup, 2nd, 4th & 5th Axis Setup
  servostepperSetup();
  //axis1stHoming();
  gripperHoming();
}


void loop() {
  manualControl();
}

// -------------------** SETUP **-------------------
void roboclawSetup() { // Roboclaw Setup 2nd, 4th, and 5th axis
  roboclaw2nd.begin(115200); // 2nd Axis
  roboclaw.begin(115200); // 4th & 5th Axis
  roboclaw2nd.ResetEncoders(0x80); // 2nd Axis
  roboclaw.ResetEncoders(0x81); // 4th & 5th Axis
}
void servostepperSetup() { // 3rd Axis Stepper Servo Motor Setup
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  digitalWrite(DIR, LOW);
  digitalWrite(PUL, LOW);
}

// -------------------** SENSOR **-------------------
int switchDetect2nd() { // Limit Switch 2nd Axis
  if (digitalRead(LIMIT_SWITCH_2ND) == HIGH) {
    Serial.print("2nd Axis Reached");
    switch2nd = 1;
  } else {
    switch2nd = 0;
  }
  return switch2nd;
}
int switchDetect3rd() { // Limit Switch 3rd Axis
  if (digitalRead(LIMIT_SWITCH_3RD) == HIGH) {
    Serial.println("3rd Axis Reached");
    switch3rd = 1;
  } else {
    switch3rd = 0;
  }
  return switch3rd;
}

//  -------------------** HOMING **-------------------
// void axis1stHoming() { // 1st Axis Homing Function
//  sparkmax.attach(sparkMaxPin);
//  Serial.print("1st Axis is ready");
// }
void gripperHoming() { // Gripper Homing Equation
 servo.attach(servoPin);
 servo.writeMicroseconds(val); // send "stop" signal to ESC.
 Serial.print("Gripper is ready");
}

// -------------------** THRUSTMASTER DATA TRANSMIT **-------------------
float parseAxisValue(const String& data) {
  int colonIndex = data.indexOf(':');
  if (colonIndex != -1) {
    String numericPart = data.substring(colonIndex + 1);
    numericPart.trim();
    return numericPart.toFloat();
  }
  return 0; // Return 0 if parsing fails
}
void parseHatValues(const String& data, float& hatX, float& hatY) {
    int openParenIndex = data.indexOf('(');
    int commaIndex = data.indexOf(',');
    int closeParenIndex = data.indexOf(')');
    if (openParenIndex != -1 && commaIndex != -1 && closeParenIndex != -1) {
        String hatXStr = data.substring(openParenIndex + 1, commaIndex);
        String hatYStr = data.substring(commaIndex + 1, closeParenIndex);
        hatX = hatXStr.toFloat();
        hatY = hatYStr.toFloat();
    } else {
        hatX = 0;
        hatY = 0;
    }
}

// -------------------** MANUAL CONTROL **-------------------
void moveServoGradually(int targetPosition) { // Servo Button Control
    int step = (targetPosition > pos) ? 1 : -1;
    while (pos != targetPosition) {
        pos += step;
        new_val = pos + val;
        servo.writeMicroseconds(new_val);
        delay(5); // Adjust for smoother or faster movement
    }
}

void manualControl() {   
    static String receivedData = ""; // Accumulator for received characters
    static float lastSpeedAxisValue = 0; // Store the last Axis 1 value for debounce logic
    static float last1stAxisValue = 0; // Store the last Axis 1 value for debounce logic
    static float last2ndAxisValue = 0; // Store the last Axis 1 value for debounce logic
    static float last3rdAxisValue = 0; // Store the last Axis 1 value for debounce logic
    static float last4thAxisValue = 0; // Store the last Axis 1 value for debounce logic
    static float last5thAxisValue = 0; // Store the last Axis 1 value for debounce logic
    static float lastGripperValue = 1600; // Store the last Axis 1 value for debounce logic
    static bool button0Pressed = false; // Flag for Button 2 state
    static bool button1Pressed = false; // Flag for Button 3 state
    static bool button2Pressed = false; // Flag for Button 2 state
    static bool button3Pressed = false; // Flag for Button 3 state
    while (Serial.available()) {
      char c = Serial.read(); // Read input from Thrustmaster Controller
      const int deadZone = 0.3; // Dead Zone on Thrustmaster Controller
       // Use slider to adjust the speed
      if (c == '\n') { // End of a message       
        /* -------------------- AXIS SPEED (SLIDER) ---------------------- */
        if (receivedData.startsWith("Axis 3:")) { // Receive data from "Axis 0:" from Thrustmaster
          float axisSpeed = parseAxisValue(receivedData);
          // Velocity Actuating
          if (!isnan(axisSpeed) && abs(axisSpeed - lastSpeedAxisValue) > 0.01) {
            speedPercent = abs(map(axisSpeed, -1, 1, 0, 1));
            lastSpeedAxisValue = axisSpeed;
          }
        }
        
        /* -------------------- 1ST AXIS ---------------------- */
        if (receivedData.startsWith("Axis 2:")) { // Receive data from "Axis 0:" from Thrustmaster
          float axisValue1 = parseAxisValue(receivedData);
          // Deadzone
          if (abs(axisValue1) < deadZone) {
            axisValue1 = 0;
          }
          // Velocity Actuating
          if (!isnan(axisValue1) && abs(axisValue1 - last1stAxisValue) > 0.01) {
            float speed1st = map((axisValue1), -1, 1, 1000, 2000); // Sparkmax PWM Control Interface. 1000 is full CCW, 2000 is full CW
            sparkmax.writeMicroseconds(lastGripperValue);
            last1stAxisValue = axisValue1;
          }
        }

        /* -------------------- 2ND AXIS ---------------------- */
        if (receivedData.startsWith("Axis 0:")) {
          float axisValue2 = parseAxisValue(receivedData);
          // Deadzone
          if (abs(axisValue2) < deadZone) {
            axisValue2 = 0;
          }
          // Velocity Actuating
          if (!isnan(axisValue2) && abs(axisValue2 - last2ndAxisValue) > 0.01) {
            float speed2nd = map(abs(axisValue2), 0, 1, 0, 64)*speedPercent; // ROBOCLAW DOESN'T TAKES NEGATIVE, put absolute for axisValue2
            if (axisValue2 > 0) {
              roboclaw2nd.ForwardM1(ROBOCLAW_ADDRESS_2ND, speed2nd);
            } else {
              roboclaw2nd.BackwardM1(ROBOCLAW_ADDRESS_2ND, speed2nd);
            }
            last2ndAxisValue = axisValue2;
          }
        }

        /* -------------------- 3RD AXIS ---------------------- */
        if (receivedData.startsWith("Axis 1:")) { // Receive data from "Axis 1:" from Thrustmaster
           float axisValue3 = parseAxisValue(receivedData);
           // Deadzone
           if (abs(axisValue3) < 0.3) {
               axisValue3 = 0;
           }
          // Velocity Actuating
          if (!isnan(axisValue3) && abs(axisValue3 - last3rdAxisValue) > 0.01) {
            if (axisValue3 >= 0) {
                // Rotate clockwise
                digitalWrite(DIR, HIGH);  // Set direction
                int speed3rd = map(axisValue3, 0, 1, 0, 255)*speedPercent;
                analogWrite(PUL, speed3rd);

            } 
            if (axisValue3 < 0){
                // Rotate counterclockwise
                digitalWrite(DIR, LOW);  // Set direction
                int speed3rd = map(axisValue3, -1, 0, 255, 0)*speedPercent;
                analogWrite(PUL, speed3rd);
            }
            last3rdAxisValue = axisValue3;
          }
        }

        /* -------------------- 4TH & 5TH AXIS ---------------------- */
        if (receivedData.startsWith("Hat 0:")) {
          float hatX, hatY;
          parseHatValues(receivedData, hatX, hatY);
          // Process hat switch movement
          if (hatX != last4thAxisValue || hatY != last5thAxisValue) {
              // Map hat switch movement to control 4th and 5th axis
              float speed4th = map(abs(hatX), 0, 1, 0, 64)*speedPercent;
              float speed5th = map(abs(hatY), 0, 1, 0, 64)*speedPercent;
              if (hatX > 0 && hatY == 0) {
                roboclaw.ForwardM1(ROBOCLAW_ADDRESS_4TH_5TH, speed4th);
                roboclaw.BackwardM2(ROBOCLAW_ADDRESS_4TH_5TH, speed4th);
              } else if (hatX < 0 && hatY == 0) {
                roboclaw.BackwardM1(ROBOCLAW_ADDRESS_4TH_5TH, speed4th);
                roboclaw.ForwardM2(ROBOCLAW_ADDRESS_4TH_5TH, speed4th);
              } else if (hatY > 0 && hatX == 0) {
                roboclaw.ForwardM1(ROBOCLAW_ADDRESS_4TH_5TH, speed5th);
                roboclaw.ForwardM2(ROBOCLAW_ADDRESS_4TH_5TH, speed5th);
              } else if (hatY < 0 && hatX == 0) {
                roboclaw.BackwardM1(ROBOCLAW_ADDRESS_4TH_5TH, speed5th);
                roboclaw.BackwardM2(ROBOCLAW_ADDRESS_4TH_5TH, speed5th);
              } else {
                roboclaw.BackwardM1(ROBOCLAW_ADDRESS_4TH_5TH, 0);
                roboclaw.BackwardM2(ROBOCLAW_ADDRESS_4TH_5TH, 0);
              }
              
              last4thAxisValue = hatX;
              last5thAxisValue = hatY;
          }
        }

        /* -------------------- Gripper ---------------------- */
        if (receivedData.startsWith("Button 0:")) {
            float gripperValue = parseAxisValue(receivedData);
            button0Pressed = (gripperValue == 1); // Check if Button 0 is pressed
        }
        if (receivedData.startsWith("Button 1:")) {
            float gripperValue = parseAxisValue(receivedData);
            button1Pressed = (gripperValue == 1); // Check if Button 1 is pressed
        }
        if (button0Pressed && !button1Pressed) {
            lastGripperValue += 0.1; // Increase for finer step but lagger response
            lastGripperValue = constrain(lastGripperValue, 1200, 1900); // Constrain PWM values, max range is 1000 to 2000
            servo.writeMicroseconds(lastGripperValue);
        } else if (!button0Pressed && button1Pressed) {
            lastGripperValue -= 0.1; // Decrease for finer step but lagger response
            lastGripperValue = constrain(lastGripperValue, 1200, 1900); // Constrain PWM values, max range is 1000 to 2000
            servo.writeMicroseconds(lastGripperValue);
        }

        receivedData = ""; // Clear for the next message
        } else if (c != '\r') { // Ignore carriage return characters
            receivedData += c; // Accumulate the next character
        }
    }
}
