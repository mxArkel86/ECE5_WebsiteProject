/* ************************************************************************************************* */
// * UCSD ECE 5 Lab 4 Code: Line Following Robot with PID * //
/* ************************************************************************************************* */

//   This is code for your PID controlled line following robot.
//
//
//
//
// Code Table of Contents
//  1)  Declare Variables     - declares many variables as global variables so each variable can be accessed from every function
//  2)  Setup (Main)          - runs once at beginning when you press button on arduino or motor drive or when you open serial monitor
//  3)  Loop  (Main)          - loops forever calling on a series of functions
//  4)  Calibration           - makes white = 0 and black = 100 (a few seconds to prep, a few seconds on white, a few seconds to move to black, a few seconds of black)
//  5)  Read Potentiometers   - reads each potentiometer
//  6)  Run Motors            - runs motors
//  7)  Read Photoresistors   - reads each photoresistor
//  8)  Calculate Error       - calculate error from photoresistor readings
//  9)  PID Turn              - takes the error and implements PID control
//  10) Print                 - used for debugging but should comment out when not debugging because it slows down program

// ************************************************************************************************* //
// Declare Variables

// Variables and Libaries for Motor
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define totalPhotoResistors 7 // Defining how many photoresistors are used, modify this if more or less are used

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1); // you may switch 1 with 2 if needed (if you see motors responding to error in the opposite way they should be.
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);

int M1Sp = 20, M2Sp = 20; // this is the nominal speed for the motors when not using potentiometer
int M1SpeedtoMotor, M2SpeedtoMotor;

// Variables for Potentiometer
const int S_pin = A0; //proportional control
const int P_pin = A1; //proportional control
const int I_pin = A2; //integral control
const int D_pin = A3; //derivative control
int SpRead = 0;
int Sp;         //Speed Increase
int kPRead = 0; //proportional gain
int kIRead = 0; //integral gain
int kDRead = 0; //derivative gain

// Variables for Light Sensors
int LDR_Pin[totalPhotoResistors] = {A8, A9, A10, A11, A12, A13, A14}; // Many arrays are used in this code to simplify it
int LDR[totalPhotoResistors];                                         // these are variables that have multiple elements to each variable name
                                                                      // LDR_Pin hold 7 values and A8 is the 0th element and A11 is the 4th element

// Calibration Variables
int led_Pin = 13;              /// This is a led set up to indicate what part of the calibration you are on.
float Mn[totalPhotoResistors]; ///     You could also connect a larger/more visible LED to Pin 13 or other digital pin
float Mx[totalPhotoResistors];
float LDRf[totalPhotoResistors] = {0., 0., 0., 0., 0., 0., 0.};

int MxRead;
int MxIndex;
float AveRead;
int CriteriaForMax;
float WeightedAve;
int im0, im1, im2;

// For Motor/Control
int Turn, M1P = 0, M2P = 0;
float error, lasterror = 0, sumerror = 0;
float kP, kI, kD;

// ************************************************************************************************* //
// setup - runs once
void setup() {

  Serial.begin(9600);       // For serial communication set up
  AFMS.begin();             // For motor setup
  pinMode(led_Pin, OUTPUT); // Note that all analog pins used are INPUTs by default so don't need pinMode

  Calibrate(); // Calibrate black and white sensing

  ReadPotentiometers(); // Read potentiometer values (Sp, P, I, & D)

  delay(2000);

  RunMotors(); // Starts motors forward and strait depending on Sp (Speed from potentiometer) and M1Sp/M2Sp (Nominal values)

} // end setup()

// ************************************************************************************************* //
// loop - runs/loops forever
void loop() {

  /** FIX ME: What function should we call to read potentiometers? **/(); // Only if you want to see Potentiometers working in set up as you run the line following

  /** FIX ME: What function should we call to read photo resistors? **/(); // Read photoresistors and map to 0-100 based on calibration

  /** FIX ME: What function should we call to calculate errors? **/();

  PID_Turn(); // PID Control and Output to motors to turn

  /** FIX ME: What function should we call to run the motors **/(); // Uses info from

  //   Print();          // Print values to serial monitor  //currently commented out but could be good for debugging =)

} // end loop()

// ************************************************************************************************* //
// function to calibrate
void Calibrate() {

  // Calibration
  int numberOfMeasurements = 30;

  // White Calibration
  CalibrateHelper(numberOfMeasurements, false);

  // Time to move from White to Black Surface
  // Black Calibration
  CalibrateHelper(numberOfMeasurements, true);
} // end Calibrate()

void CalibrateHelper(int numberOfMeasurements, boolean ifCalibratingBlack) {
  // wait to make sure in position
  for (int i = 0; i < 4; i++) {
    digitalWrite(led_Pin, HIGH); // turn the LED on
    delay(100);                  // wait for 0.1 seconds
    digitalWrite(led_Pin, LOW);  // turn the LED off
    delay(900);                  // wait for 0.9 seconds
  }
  for (int i = 0; i < numberOfMeasurements; i++) {
    digitalWrite(led_Pin, HIGH); // turn the LED on
    delay(100);                  // wait for 0.1 seconds
    digitalWrite(led_Pin, LOW);  // turn the LED off
    delay(100);                  // wait for 0.1 seconds

    for (int pin = 0; pin < totalPhotoResistors; pin++) {
      LDRf[pin] = LDRf[pin] + (float)analogRead(LDR_Pin[pin]);
      delay(2);
    }
  }
  for (int pin = 0; pin < totalPhotoResistors; pin++) {
    if (ifCalibratingBlack) {// updating cooresponding arry based on if we are calibrating black or white
      Mx[pin] = round(LDRf[pin] / (float)numberOfMeasurements); // take average
    }
    else {
      Mn[pin] = round(LDRf[pin] / (float)numberOfMeasurements); // take average
    }
    LDRf[pin] = 0.0;
  }
}

// ************************************************************************************************* //
// function to read and map values from potentiometers
void ReadPotentiometers() {
  // Call on user-defined function to read Potentiometer values
  SpRead = /** FIX ME: We want to read a potentiometer for S_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100. (Consult 4.1 if help is needed) **/
  kPRead = /** FIX ME: We want to read a potentiometer for P_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100. (Consult 4.1 if help is needed) **/
  kIRead = /** FIX ME: We want to read a potentiometer for I_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100. (Consult 4.1 if help is needed) **/
  kDRead = /** FIX ME: We want to read a potentiometer for D_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100. (Consult 4.1 if help is needed) **/
} // end ReadPotentiometers()

// function to read and map values from potentiometers
int ReadPotentiometerHelper(int pin, int min_resolution, int max_resolution, int min_potentiometer, int max_potentiometer) {
  return map(analogRead(pin), min_resolution, max_resolution, min_potentiometer, max_potentiometer);
}

// ************************************************************************************************* //
// function to start motors using nominal speed + speed addition from potentiometer
void RunMotors() {
  M1SpeedtoMotor = min(M1Sp + Sp + M1P, 255); // limits speed to 255
  M2SpeedtoMotor = min(M2Sp + Sp + M2P, 255); // remember M1Sp & M2Sp is defined at beginning of code (default 60)

  runMotorAtSpeed(Motor1, M1SpeedtoMotor, FORWARD); // run motor 1
  runMotorAtSpeed(Motor2, M2SpeedtoMotor, FORWARD); // run motor 2
} // end RunMotors()

// A function that commands a specified motor to move towards a given direction at a given speed
void runMotorAtSpeed(Adafruit_DCMotor *motor, int speed, int direction) {
  motor->setSpeed(abs(speed)); // sets the speed of the motor from arguments
  // swap direction if speed is negative
  if (speed < 0) {
    if (direction == FORWARD) {
      motor->run(BACKWARD); // sets the direction of the motor from arguments
    }
    else if (direction == BACKWARD) {
      motor->run(FORWARD); // sets the direction of the motor from arguments
    }
  }
  else {
    motor->run(direction); // sets the direction of the motor from arguments
  }
}

// ************************************************************************************************* //
// function to read photo resistors, map from 0 to 100, and find darkest photo resitor (MxIndex)
void ReadPhotoResistors() {
  for (int Li = 0; Li < totalPhotoResistors; Li++) {
    LDR[Li] = map(analogRead(LDR_Pin[Li]), Mn[Li], Mx[Li], 0, 100);
    delay(2);
  }
} // end ReadPhotoResistors()

// ************************************************************************************************* //
// Calculate error from photoresistor readings
void CalcError() {
  MxRead = -99;
  AveRead = 0.0;
  for (int i = 0; i < totalPhotoResistors; i++) {
    if (MxRead < LDR[i]) {
      MxRead = LDR[i];
      MxIndex = -1 * (i - 3);
      im1 = (float)i;
    }
    AveRead = AveRead + (float)LDR[i] / (float)totalPhotoResistors;
  }
  CriteriaForMax = 2; // max should be at least twice as big as the other values
  if (MxRead > CriteriaForMax * AveRead) {
    // im0 stands for the photoresistor on the left of im1 and im2 stands for the photoresistor on the right of im1
    im0 = im1 - 1;
    im2 = im1 + 1;
    float numerator = (float)(LDR[im1] * im1) + (float)(im1 != 0 ? (LDR[im0] * im0) : 0) + (float)(im1 != 6 ? (LDR[im2] * im2) : 0);
    float denominator = (float)((LDR[im1]) + (float)(im0 != 0?(LDR[im0]):0) + (float)(im2 != 6?(LDR[im2]):0);
    WeightedAve = ((float)numerator)/denominator;

    error = -1 * (WeightedAve - 3); // we set the error variable from our calculation
  }
} // end CalcError()

// ************************************************************************************************* //
// function to make a turn ( a basic P controller)
void PID_Turn() {
  // *Read values are between 0 and 100, scale to become PID Constants
  kP = (float)kPRead / 1.;    // each of these scaling factors can change depending on how influential you want them to be
  kI = (float)kIRead / 1000.; // the potentiometers will also scale them
  kD = (float)kDRead / 100.;
  // error holds values from -3 to 3

  Turn = error * kP + sumerror * kI + (error - lasterror) * kD; //PID!!!!!

  sumerror = sumerror + error;
  if (sumerror > 5) {
    sumerror = 5;
  } // prevents integrator wind-up
  else if (sumerror < -5) {
    sumerror = -5;
  }

  lasterror = error;

  if      (Turn < 0) { M1P = Turn; M2P = -Turn;}   //One motor becomes slower and the other faster
  else if (Turn > 0) {M1P = Turn ; M2P = -Turn;} 
  else               {M1P = 0    ; M2P = 0;} 
} // end PID_Turn()

// ************************************************************************************************* //
// function to print values of interest
void Print() {
  Serial.print(SpRead);
  Serial.print(" "); // Initial Speed addition from potentiometer
  Serial.print(kP);
  Serial.print(" "); // PID values from potentiometers after scaling
  Serial.print(kI);
  Serial.print(" ");
  Serial.print(kD);
  Serial.print("    ");

  for (int i = 0; i < totalPhotoResistors; i++) { // Each photo resistor value is shown
    // Printing the photo resistor reading values one by one
    Serial.print(LDR[i]);
    Serial.print(" ");
  }
  Serial.print("   "); // This just prints a gap

  Serial.print(MxRead);
  Serial.print(" "); // the maximum value from the photo resistors is shown again
  Serial.print(MxIndex);
  Serial.print("    "); // this is the index of that maximum (0 through 6) (aka which element in LDR)
  Serial.print(error);
  Serial.print("    "); // this will show the calculated error (-3 through 3)

  Serial.print(M1SpeedtoMotor);
  Serial.print(" ");              // This prints the arduino output to each motor so you can see what the values (0-255)
  Serial.println(M2SpeedtoMotor); //  that are sent to the motors would be without actually needing to power/run the motors

  // delay(100); //just here to slow down the output for easier reading if wanted
  // ensure delay is commented when actually running your robot or this will slow down sampling too much
} // end Print()
