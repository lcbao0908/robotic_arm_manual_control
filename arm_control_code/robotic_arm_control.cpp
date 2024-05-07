#include "RoboClaw.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Encoder.h>

// MOTOR SETUP

// ------------------- ** SWITCH SETUP ** --------------------
int switch2nd;
int switch3rd;
#define LIMIT_SWITCH_2ND 7
#define LIMIT_SWITCH_3RD 8

// ------------------- ** 1st Axis ODESC and NEO Motor ** ---------------------
byte sparkMaxPin = 14;
Servo sparkmax;

// ------------------- ** 2nd Axis & ROBOCLAW ** ---------------------
RoboClaw roboclaw2nd = RoboClaw(&Serial2, 10000);
#define ROBOCLAW_ADDRESS_2ND 81

// ------------------- ** 3rd Axis Stepper Servo Motor ** ---------------------
#define PUL 36
#define DIR 38

// ------------------- ** 4th and 5th Axis GoBILDA & ROBOCLAW ** ---------------------
RoboClaw roboclaw = RoboClaw(&Serial1, 10000);
#define ROBOCLAW_ADDRESS_4TH_5TH 80

// ------------------- ** Gripper ** ---------------------
byte servoPin = 13;
Servo servo;

// ENCODER SETUP

// ------------------- ** 9 DOF Adafruit BNO055 IMU ** ---------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55);
double roll, pitch, yaw;

// ------------------- ** 2nd Axis AMT102-V Incremental Encoder ** ---------------------
const int pinA_2nd = 22;   // Connect to Pin A of the encoder
const int pinB_2nd = 23;   // Connect to Pin B of the encoder
const int pinZ_2nd = 24;   // Connect to Z index of the encoder
Encoder Encoder2nd(pinA_2nd, pinB_2nd);
long encoder_count_2nd;
double AngleIncrement2nd = 360.0/8192; // 8192 CPR, AMT102-V Incremental Encoder
double theta2nd, theta2ndHome; 

// ------------------- ** 3rd Axis AMT102-V Incremental Encoder ** ---------------------
const int pinA_3rd = 25;   // Connect to Pin A of the encoder
const int pinB_3rd = 26;   // Connect to Pin B of the encoder
const int pinZ_3rd = 27;   // Connect to Z index of the encoder
Encoder Encoder3rd(pinA_3rd, pinB_3rd);
int encoder_count_3rd;
double AngleIncrement3rd = 360.0/8192; // 8192 CPR, AMT102-V Incremental Encoder
double theta3r, theta3rdHome;

// ------------------- ** 4th and 5th Axis Encoder GoBILDA & ROBOCLAW ** ---------------------
int encoder_count_4th;
int encoder_count_5th;
double AngleIncrement4th_5th = 360.0/1538; // 1538 CPR, GoBilda Motor 5204-8002-0014
double theta4th, theta4thHome;
double theta5th, theta5thHome;

// MATH PROCESSING
double current_theta2nd, current_theta3rd, current_theta4th, current_theta5th;
double target_2ndAngle, target_3rdAngle;
double x, y; // XY Plane of Robotic Arm
double L1 = 450; // 2nd Axis to 3rd Axis Link Length (mm)
double L2 = 322.765; // 3rd Axis to 4th/5th Axis Link Length (mm)
double L3 = 230; // 4th/5th Axis to Gripper Tip Link Length (mm)


// ------------------- ** Kinematic Global Variables ** --------------------- 
void setup() {
  Serial.begin(115200);
  pinMode(LIMIT_SWITCH_2ND, INPUT);
  pinMode(LIMIT_SWITCH_3RD, INPUT);
  roboclawSetup(); // Roboclaw Setup, 2nd, 4th & 5th Axis Setup
  imuSensorSetup(); // IMU Setup 
  axis1stHoming();
  axis2ndHoming();
  axis3rdHoming();
  gripperHoming();
  encoderReset();
}

void loop() {
  // Grab BNO055 IMU and Sensor Data
  bno055sensor();
  encoderData();

  if (Serial.available() > 0) {
    String message = Serial.read();
    int mode = message[0];  // Read the mode command

    switch (mode) {
      case '0': // Manual Control Mode
        manualControl();
        break;
      case '1': // Inverse Kinematic Control Mode
        IKEControl();
        break;
    }
  }
}

// -------------------** SETUP **-------------------
void roboclawSetup() { // Roboclaw Setup 2nd, 4th, and 5th axis
  roboclaw2nd.begin(115200); // 2nd Axis
  roboclaw.begin(115200); // 4th & 5th Axis
  roboclaw2nd.ResetEncoders(0x81); // 2nd Axis
  roboclaw.ResetEncoders(0x80); // 4th & 5th Axis
}
void imuSensorSetup() { // BNO055 Absolute Sensor Setup
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
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

void bno055sensor() { // BNO055 Absolute Sensor
  sensors_event_t event; 
  bno.getEvent(&event);
  yaw = event.orientation.x;
  pitch = event.orientation.y;
  roll = event.orientation.z;
}
void encoderData() { // Axis Encoder Data Read
  encoder_count_2nd = Encoder2nd.read();
  encoder_count_3rd = Encoder3rd.read();
  encoder_count_4th = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS_4TH_5TH);
  encoder_count_5th = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS_4TH_5TH);
}
void encoderReset() { // Axis Encoder Data Reset
  Encoder2nd.write(0);
  Encoder3rd.write(0);
  roboclaw.ResetEncoders(0x80);
  roboclaw.ResetEncoders(0x81);
}

//  -------------------** HOMING **-------------------
void axis1stHoming() { // 1st Axis Homing Function
 sparkmax.attach(sparkMaxPin);
 Serial.print("1st Axis is ready");
}
void axis2ndHoming() { // 2nd Axis Homing Function
  roboclaw.ForwardM1(ROBOCLAW_ADDRESS_2ND, 32);
  // Motor move until reaches limit switch
  do {
    switch2nd = switchDetect2nd(); // Assign the return value of switchDetect2nd to switch2nd
    delay(100);  // Short delay to avoid too frequent polling
  } while (switch2nd == 0);
  
  roboclaw.ForwardM1(ROBOCLAW_ADDRESS_2ND, 0);  // Stop the motor
  theta2ndHome = 60;  // Degrees, starting angle of 2nd axis
  return theta2ndHome;
}
void axis3rdHoming() { // 3rd Axis Homing Function
  digitalWrite(DIR, LOW); 
  analogWrite(PUL, 125);
  // Motor move until reaches limit switch
  do {
    switch3rd = switchDetect3rd(); // Assign the return value of switchDetect2nd to switch2nd
    switchDetect3rd();
    delay(100);  // Short delay to avoid too frequent polling
  } while (switch3rd == 0);
  analogWrite(PUL, 0);
  float theta3rdHome = -170;  // Degrees, starting angle of 3rd axis
  return theta3rdHome; 
}
void gripperHoming() { // Gripper Homing Equation
 servo.attach(servoPin);
 servo.writeMicroseconds(1600); // send "stop" signal to ESC.
 Serial.print("Gripper is ready");
}

// -------------------** PRINT & DEBUGGING **-------------------
void printData() {
  // Serial Plotter
  Serial.print(roll); Serial.print(",");
  Serial.print(pitch); Serial.print(",");
  Serial.print(yaw); Serial.print(",");

  // Display Data
  Serial.print("\t\tRoll: "); Serial.print(roll);
  Serial.print("\t\tPitch: "); Serial.print(pitch);
  Serial.print("\t\tYaw: "); Serial.print(yaw);
  Serial.print("\r\n");

  delay(2); //
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

// -------------------** KINEMATICS PROCESSING **-------------------
int mapTiltToSpeed(float tiltAngle, int maxMotorSpeed) { // Speed mapping for BNO055
  // Assuming tiltAngle is in degrees and can range from -10 to 10 degrees
  // Map the absolute value of tiltAngle to a motor speed
  int speed = (int)map(abs(tiltAngle), 0, 10, 0, maxMotorSpeed);

  // Adjust the mapping as needed for your application
  return speed;
} 
void roll_level() { // Auto Leveling in roll angle
  const float deadZone = 0.05; // Degrees, adjust based on your requirements
  const int maxMotorSpeed = 40; // Maximum speed of motors

  // Check if within the dead zone (i.e., close enough to level)
  if (abs(roll) <= deadZone) {
    // If within dead zone, stop motors
    roboclaw.ForwardM1(0x80, 0); // Assuming 0 speed stops the motor
    roboclaw.ForwardM2(0x80, 0);
  } else {
    // Map tilt angle to speed
    int speed = mapTiltToSpeed(roll, maxMotorSpeed); // Return roll angle as tiltAngle

    // Decide direction based on the sign of roll angle and apply speed
    if (roll > 0) {
      // If roll angle is positive, differential drive, two motor move in same direction change in roll angle
      // Tilt down until reach 0 degree of roll angle
      roboclaw.ForwardM2(0x80, speed);
      roboclaw.ForwardM1(0x80, speed);
    } else {
      // If roll angle is negative, differential drive, two motor move in same direction change in roll angle
      // Tilt up until reach 0 degree of roll angle
      roboclaw.BackwardM2(0x80, speed);
      roboclaw.BackwardM1(0x80, speed);
    }
  }
}
void pitch_level() { // Auto Leveling in pitch angle
  const float deadZone = 0.05; // Degrees, adjust based on your requirements
  const int maxMotorSpeed = 40; // Maximum speed of motors

  // Check if within the dead zone (i.e., close enough to level)
  if (abs(pitch) <= deadZone) {
    // If within dead zone, stop motors
    roboclaw.ForwardM1(0x80, 0); // Assuming 0 speed stops the motor
    roboclaw.ForwardM2(0x80, 0);
  } else {
    // Map tilt angle to speed
    int speed = mapTiltToSpeed(pitch, maxMotorSpeed); // Return pitch angle as tiltAngle

    // Decide direction based on the sign of roll angle and apply speed
    if (pitch > 0) {
      // If roll angle is positive, differential drive, two motor move in opposite direction change in pitch angle
      // Tilt clockwise until pitch angle reach 0
      roboclaw.ForwardM2(0x80, speed);
      roboclaw.BackwardM1(0x80, speed);
    } else {
      // If roll angle is negative, differential drive, two motor move in opposite direction change in pitch angle
      // Tilt counterclockwise until pitch angle reach 0
      roboclaw.BackwardM2(0x80, speed);
      roboclaw.ForwardM1(0x80, speed);
    }
  }
}
void auto_level() { // Auto Leveling combining roll and pitch
  const float deadZone = 0.05; // Degrees, adjust based on your requirements
  const int maxMotorSpeed = 40; // Maximum speed of motors
}
void encoderAngle() {
  encoderData();
  current_theta2nd = theta2ndHome + AngleIncrement2nd*encoder_count_2nd; // Calculate current angle of 2nd axis encoder
  current_theta3rd = theta3rdHome + AngleIncrement3rd*encoder_count_3rd; // Calculate current angle of 3rd axis encoder
  current_theta4th = AngleIncrement4th_5th*encoder_count_4th; // Calculate angle of 4th & 5th axis encoder
  current_theta5th = AngleIncrement4th_5th*encoder_count_5th; // Calculate angle of 4th & 5th axis encoder
}
void forwardKinematicSolver(double theta2nd, double theta3rd) { // Forward kinematic solver to record current x & y for manual control
  // x and y is the absolute coordinate of the arm system at the end of 3rd axis
  // This function is used to record the current x and y coordinate while moving ...
  // during manual control and immediately switch to inverse kinematic
  x = L1*cos(current_theta2nd*(3.1416/180)) + L2*cos((current_theta2nd + current_theta3rd)*(3.1416/180)); //+ L3*cos(theta2nd + theta3rd + roll); // X-distance in XY plane
  y = L1*sin(current_theta2nd*(3.1416/180)) + L2*sin((current_theta2nd + current_theta3rd)*(3.1416/180)); //+ L3*sin(theta2nd + theta3rd + roll); // Y-distance in XY plane
}
double pathAngle(double angle) { // Determine the most efficient path for rotation
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}
void moveAxis(double theta2i, double theta2f, double theta3i, double theta3f) {
    // Search for nearest path to rotate arm to reach desired angles
    double error2nd = pathAngle(theta2f - theta2i);
    double error3rd = pathAngle(theta3f - theta3i);

    // Map the absolute error to speed values
    int speed2nd = map(abs(error2nd), 0, 1, 0, 64); // Assuming max error is 180 degrees
    speed2nd = constrain(speed2nd, 0, 64);
    int speed3rd = map(abs(error3rd), 0, 1, 0, 255); // Assuming max error is 180 degrees
    speed3rd = constrain(speed3rd, 0, 255);
    
    // 2nd axis control
    if (error2nd > 0) {
        roboclaw.ForwardM1(ROBOCLAW_ADDRESS_2ND, speed2nd);
        // Serial.println("2nd Axis CCW, speed:", speed2nd);
    } else if (error2nd < 0) {
        roboclaw.BackwardM1(ROBOCLAW_ADDRESS_2ND, speed2nd);
        // Serial.println("2nd Axis CW, speed:", speed2nd);
    } else {
        roboclaw.ForwardM1(ROBOCLAW_ADDRESS_2ND, 0);
    }

    // 3rd axis velocity control
    if (error3rd > 0) {
        digitalWrite(DIR, HIGH);  // Set direction clockwise
        analogWrite(PUL, speed3rd);
    } else if (error3rd < 0) {
        digitalWrite(DIR, LOW); // Set direction counterclockwise
        analogWrite(PUL, speed3rd);
    } else {
        digitalWrite(DIR, HIGH);
        analogWrite(PUL, 0);
    }
}
void IKESolver(double x, double y) { // Function to solve inverse kinematic input based on x and y
    // Inverse Kinematic Solve
    double c2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
    double s2 = sqrt(1 - c2*c2);
    target_3rdAngle = -atan2(s2, c2);
    target_2ndAngle = (atan2(y,x) + atan2((L2*s2),(L1 + L2*c2)));

    // Convert to degrees and paste data
    double shoulder_angle = target_2ndAngle*(180/3.1416);
    double ankle_angle = target_3rdAngle*(180/3.1416);
    
    encoderAngle();
    moveAxis(current_theta2nd, shoulder_angle, current_theta3rd, ankle_angle);
}
void IKEControl() {   // Inverse Kinematic Control Mode
    static String receivedData = ""; // Accumulator for received characters
    static float last1stAxisValue = 0; // Store the last Axixs 1 value for debounce logic
    static float lastVerticalValue = 0; // Store the last Y-coordinate value for debounce logic
    static float lastHorizontalValue = 0; // Store the last X-coordinate value for debounce logic
    static float lastGripperValue = 0; // Store the last Axis 1 value for debounce logic
    static bool button0Pressed = false; // Flag for Button 2 state
    static bool button1Pressed = false; // Flag for Button 3 state
    static bool button2Pressed = false; // Flag for Button 2 state
    static bool button3Pressed = false; // Flag for Button 3 state
    
    while (Serial.available()) {
      char c = Serial.read(); // Read input from Thrustmaster Controller
      digitalWrite(LED_BUILTIN, HIGH); // Indicate data reception on Teensy
      const int deadZone = 0.2; // Dead Zone on Thrustmaster Controller

      if (c == '\n') { // End of a message
                /* -------------------- 1ST AXIS ---------------------- */
        if (receivedData.startsWith("Button 2:")) {
            float buttonValue = parseAxisValue(receivedData);
            button2Pressed = (buttonValue == 1); // Check if Button 2 is pressed
        }
        if (receivedData.startsWith("Button 3:")) {
            float buttonValue = parseAxisValue(receivedData);
            button3Pressed = (buttonValue == 1); // Check if Button 3 is pressed
        }
        if (button2Pressed && !button3Pressed) { // Adjust PWM if button is pressed and hole
          if (last1stAxisValue < 1525) {
            last1stAxisValue += 25; // Escaping PWM deadzone of Sparkmax 1475 -> 1525
          }
          last1stAxisValue += 1; // Increase PWM to increase velocity
          last1stAxisValue = constrain(last1stAxisValue, 1300, 1700); // Constrain PWM values, max range is 1000 to 2000
          sparkmax.writeMicroseconds(last1stAxisValue);
        } else if (!button2Pressed && button3Pressed) {
          if (last1stAxisValue > 1475) {
          last1stAxisValue -= 25; // Escaping PWM deadzone of Sparkmax 1475 -> 1525
          }
          last1stAxisValue -= 1; // Decrease PWM by 50 to decrease velocity
          last1stAxisValue = constrain(last1stAxisValue, 1300, 1700); // Constrain PWM values, max range is 1000 to 2000
          sparkmax.writeMicroseconds(last1stAxisValue);
        } else if (!button2Pressed && !button3Pressed) {
          last1stAxisValue = 1500; // Reset to neutral when neither button is pressed
          sparkmax.writeMicroseconds(1500);
        }

        /* -------------------- HORIZONTAL ---------------------- */
        if (receivedData.startsWith("Axis 0:")) { // Receive data from "Axis 0:" from Thrustmaster
          float HorizontalValue = parseAxisValue(receivedData);
          // Deadzone
          if (abs(HorizontalValue) < deadZone) {
            HorizontalValue = 0;
          }
          if (!isnan(HorizontalValue) && abs(HorizontalValue - lastHorizontalValue) > 0.01) { // Adjust threshold as needed
            x += HorizontalValue;
            lastHorizontalValue = HorizontalValue;
            IKESolver(x,y);
          }
        }

        /* -------------------- VERTICAL ---------------------- */
        if (receivedData.startsWith("Axis 1:")) { // Receive data from "Axis 0:" from Thrustmaster
          float VerticalValue = parseAxisValue(receivedData);
          // Deadzone
          if (abs(VerticalValue) < deadZone) {
            VerticalValue = 0;
          }
          if (!isnan(VerticalValue) && abs(VerticalValue - lastVerticalValue) > 0.01) { // Adjust threshold as needed
            y += VerticalValue;
            lastVerticalValue = VerticalValue;
            IKESolver(x,y);
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
            lastGripperValue += 1; // Increase PWM to increase velocity
            lastGripperValue = constrain(lastGripperValue, 1200, 1900); // Constrain PWM values, max range is 1000 to 2000
            sparkmax.writeMicroseconds(lastGripperValue);
        } else if (!button0Pressed && button1Pressed) {
            lastGripperValue -= 1; // Decrease PWM by 50 to decrease velocity
            lastGripperValue = constrain(lastGripperValue, 1200, 1900); // Constrain PWM values, max range is 1000 to 2000
            sparkmax.writeMicroseconds(lastGripperValue);
        }

        
        receivedData = ""; // Clear for the next message
        digitalWrite(LED_BUILTIN, LOW); // Turn off the LED after processing
        } else if (c != '\r') { // Ignore carriage return characters
            receivedData += c; // Accumulate the next character
        }
    }
}

// -------------------** MANUAL CONTROL **-------------------
void manualControl() {   
    static String receivedData = ""; // Accumulator for received characters
    static float last1stAxisValue = 0; // Store the last Axis 1 value for debounce logic
    static float last2ndAxisValue = 0; // Store the last Axis 1 value for debounce logic
    static float last3rdAxisValue = 0; // Store the last Axis 1 value for debounce logic
    static float last4thAxisValue = 0; // Store the last Axis 1 value for debounce logic
    static float last5thAxisValue = 0; // Store the last Axis 1 value for debounce logic
    static float lastGripperValue = 0; // Store the last Axis 1 value for debounce logic
    static bool button0Pressed = false; // Flag for Button 2 state
    static bool button1Pressed = false; // Flag for Button 3 state
    static bool button2Pressed = false; // Flag for Button 2 state
    static bool button3Pressed = false; // Flag for Button 3 state
    
    while (Serial.available()) {
      char c = Serial.read(); // Read input from Thrustmaster Controller
      digitalWrite(LED_BUILTIN, HIGH); // Indicate data reception on Teensy
      const int deadZone = 0.3; // Dead Zone on Thrustmaster Controller

      if (c == '\n') { // End of a message
        /* -------------------- 1ST AXIS ---------------------- */
        if (receivedData.startsWith("Button 2:")) {
            float buttonValue = parseAxisValue(receivedData);
            button2Pressed = (buttonValue == 1); // Check if Button 2 is pressed
        }
        if (receivedData.startsWith("Button 3:")) {
            float buttonValue = parseAxisValue(receivedData);
            button3Pressed = (buttonValue == 1); // Check if Button 3 is pressed
        }
        if (button2Pressed && !button3Pressed) { // Adjust PWM if button is pressed and hole
          if (last1stAxisValue < 1525) {
            last1stAxisValue += 25; // Escaping PWM deadzone of Sparkmax 1475 -> 1525
          }
          last1stAxisValue += 1; // Increase PWM to increase velocity
          last1stAxisValue = constrain(last1stAxisValue, 1300, 1700); // Constrain PWM values, max range is 1000 to 2000
          sparkmax.writeMicroseconds(last1stAxisValue);
        } else if (!button2Pressed && button3Pressed) {
          if (last1stAxisValue > 1475) {
          last1stAxisValue -= 25; // Escaping PWM deadzone of Sparkmax 1475 -> 1525
          }
          last1stAxisValue -= 1; // Decrease PWM by 50 to decrease velocity
          last1stAxisValue = constrain(last1stAxisValue, 1300, 1700); // Constrain PWM values, max range is 1000 to 2000
          sparkmax.writeMicroseconds(last1stAxisValue);
        } else if (!button2Pressed && !button3Pressed) {
          last1stAxisValue = 1500; // Reset to neutral when neither button is pressed
          sparkmax.writeMicroseconds(1500);
        }

        /* -------------------- 2ND AXIS ---------------------- */
        if (receivedData.startsWith("Axis 0:")) { // Receive data from "Axis 0:" from Thrustmaster
          float axisValue2 = parseAxisValue(receivedData);
          switchDetect2nd();
          // Deadzone
          if (abs(axisValue2) < deadZone) {
            axisValue2 = 0;
          }
          // Velocity Actuating
          if (!isnan(axisValue2) && abs(axisValue2 - last2ndAxisValue) > 0.01) {
            float speed2nd = map(axisValue2, 0, 1, 0, 64); // Forward PWM mapping
            if (axisValue2 > 0 && switch2nd == 0) {
              roboclaw.ForwardM1(ROBOCLAW_ADDRESS_2ND, speed2nd);
              //Serial2.println("Sending Forward command to 2nd Axis");
            } else if (axisValue2 < 0 && switch2nd == 0) {
              roboclaw.BackwardM1(ROBOCLAW_ADDRESS_2ND, speed2nd);
              //Serial2.println("Sending Backward command to 2nd Axis");
            } else {
              roboclaw.ForwardM1(ROBOCLAW_ADDRESS_2ND, 0);
              //Serial2.println("2nd Axis Park");
            }
            last2ndAxisValue = axisValue2; // Update the last known axis value
          }
        }

        /* -------------------- 3RD AXIS ---------------------- */
        if (receivedData.startsWith("Axis 1:")) { // Receive data from "Axis 1:" from Thrustmaster
           float axisValue3 = parseAxisValue(receivedData);
           switchDetect3rd();
           // Deadzone
           if (abs(axisValue3) < deadZone) {
               axisValue3 = 0;
           }
          // Velocity Actuating
          if (!isnan(axisValue3) && abs(axisValue3 - last3rdAxisValue) > 0.01) {
              if (axisValue3 > 0 && switch3rd == 0) {
                  // Rotate clockwise
                  digitalWrite(DIR, HIGH);  // Set direction
                  float speed3rd = map(axisValue3, 0, 1, 0, 255);
                  analogWrite(PUL, speed3rd);
              } else if (axisValue3 < 0 && switch3rd == 0) {
                  // Rotate counterclockwise
                  digitalWrite(DIR, LOW); 
                  float speed3rd = map(axisValue3, -1, 0, 255, 0);
                  analogWrite(PUL, speed3rd);
              } else {
                  digitalWrite(DIR, HIGH);
                  float speed3rd = 0;
                  analogWrite(PUL, speed3rd);
              }
              last3rdAxisValue = axisValue3;  // Update the last axis value for the next comparison
          }
        }

        /* -------------------- 4TH AXIS ---------------------- */
        if (receivedData.startsWith("Axis 2:")) { // Receive data from "Axis 2:" from Thrustmaster
          float axisValue4 = parseAxisValue(receivedData);
          // Deadzone
          if (abs(axisValue4) < deadZone) {
            axisValue4 = 0;
          }
          // Velocity Actuating
          if (!isnan(axisValue4) && abs(axisValue4 - last4thAxisValue) > 0.01) {
            float speed4th = map(axisValue4, 0, 1, 0, 64); // Forward PWM mapping
            if (axisValue4 > 0) {
              roboclaw.ForwardM1(ROBOCLAW_ADDRESS_4TH_5TH, speed4th);
              roboclaw.BackwardM2(ROBOCLAW_ADDRESS_4TH_5TH, speed4th);
              //Serial2.println("Sending CCW to 4th Axis");
            } else if (axisValue4 < 0) {
              roboclaw.BackwardM1(ROBOCLAW_ADDRESS_4TH_5TH, speed4th);
              roboclaw.ForwardM2(ROBOCLAW_ADDRESS_4TH_5TH, speed4th);

              //Serial2.println("Sending CW to 4th Axis");
            } else {
              roboclaw.ForwardM1(ROBOCLAW_ADDRESS_4TH_5TH, 0);
              roboclaw.BackwardM2(ROBOCLAW_ADDRESS_4TH_5TH, 0);
              //Serial2.println("2nd Axis Park");
            }
            last4thAxisValue = axisValue4; // Update the last known axis value
          }
        }

        /* -------------------- 5TH AXIS ---------------------- */
        if (receivedData.startsWith("Axis 3:")) { // Receive data from "Axis 3:" from Thrustmaster
          float axisValue5 = parseAxisValue(receivedData);
          // Deadzone
          if (abs(axisValue5) < deadZone) {
            axisValue5 = 0;
          }
          // Velocity Actuating
          if (!isnan(axisValue5) && abs(axisValue5 - last5thAxisValue) > 0.01) {
            float speed5th = map(axisValue5, 0, 1, 0, 64); // Forward PWM mapping
            if (axisValue5 > 0) {
              roboclaw.ForwardM1(ROBOCLAW_ADDRESS_4TH_5TH, speed5th);
              roboclaw.ForwardM2(ROBOCLAW_ADDRESS_4TH_5TH, speed5th);
              //Serial2.println("Sending CCW to 4th Axis");
            } else if (axisValue5 < 0) {
              roboclaw.ForwardM1(ROBOCLAW_ADDRESS_4TH_5TH, speed5th);
              roboclaw.ForwardM2(ROBOCLAW_ADDRESS_4TH_5TH, speed5th);
              //Serial2.println("Sending CW to 4th Axis");
            } else {
              roboclaw.ForwardM1(ROBOCLAW_ADDRESS_4TH_5TH, 0);
              roboclaw.ForwardM2(ROBOCLAW_ADDRESS_4TH_5TH, 0);
              //Serial2.println("2nd Axis Park");
            }
            last5thAxisValue = axisValue5; // Update the last known axis value
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
        if (button0Pressed && !button1Pressed) { // Adjust PWM if button is pressed and hole
            lastGripperValue += 1; // Increase PWM to increase velocity
            lastGripperValue = constrain(lastGripperValue, 1200, 1900); // Constrain PWM values, max range is 1000 to 2000
            sparkmax.writeMicroseconds(lastGripperValue);
        } else if (!button0Pressed && button1Pressed) {
            lastGripperValue -= 1; // Decrease PWM by 50 to decrease velocity
            lastGripperValue = constrain(lastGripperValue, 1200, 1900); // Constrain PWM values, max range is 1000 to 2000
            sparkmax.writeMicroseconds(lastGripperValue);
        }
        
        receivedData = ""; // Clear for the next message
        digitalWrite(LED_BUILTIN, LOW); // Turn off the LED after processing
        } else if (c != '\r') { // Ignore carriage return characters
            receivedData += c; // Accumulate the next character
        }
    }
}
