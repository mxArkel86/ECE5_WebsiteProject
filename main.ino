// TO-DO: ADD LOGIC FOR MODES! I.E., CHANGE CERTAIN FACTORS DEPENDING ON THE CHALLENGE 
// ALSO: FINE TUNE PID CONSTANTS! THIS WILL TAKE TRIAL/ERROR!
// ACTUALLY BOTH OF THESE WILL REQUIRE TESTING!
// ALSO THE FOURTH POTENTIOMETER'S VALUE ISNT REALLY BEING USED (GOING SLOW IS BASED)
// IF ANYONE HAS QUESTIONS ABOUT CODE ASK ME!
// -ARNAV

#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define totalPhotoResistors 7


// IR Control
#include <IRremote.h>
#define ON_OFF_KEY 0// TO-DO; ADD HEX VALUES FOR REMOTE CODES!
#define UP_ARROW 0
#define RIGHT_ARROW 0
#define LEFT_ARROW 0
#define DOWN_ARROW 0
#define CIRCLE_KEY 0
#define A_KEY 0
#define B_KEY 0
#define C_KEY 0

#define RECV_PIN 50

IRrecv irrecv(RECV_PIN);  // Initialize IR Library
decode_results results;   // Initialize IR Library

// Pins
#define S_pin A5 // PIN NUMBERS MAY NEED TO BE UPDATED !!
#define P_pin A7
#define I_pin A3
#define D_pin A15
const int RGB_Pin[3] {46,52,48};
const int PR_Pins[totalPhotoResistors] = {A8, A9, A10, A11, A12, A13, A14};

// Initial Motor Speeds. Motor speeds range from 0-255. Change speed of single motor depending on turn. 
int SpeedM1 = 255;
int SpeedM2 = 255;

int PR_Vals[totalPhotoResistors]; // Stores the color of each photoresistor

// Mode that the car is in
int mode = 0; // ranges 0-4

// The highest and lowest values for each photoresistor during calibration. 
// Each photoresistor will have different slightly different calibration
float black_values[totalPhotoResistors];
float white_values[totalPhotoResistors];

// Initialize Motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftWheel = AFMS.getMotor(1); // Motors can be switched here (1) <--> (2)
Adafruit_DCMotor *rightWheel = AFMS.getMotor(2);

// Error
float error, lastError = 0, sumerror = 0;
int threshold = 20; // if photoresistor value is under threshold, it is detecting a line

// PID values
int kP;
int kI;
int kD;

float PIDvalue;

// Adjust speed of motors using 4th potentiometer (but WE want SPEEED SICKO MOED!!1

int Sp;

void setup() {
  
  Serial.begin(9600);

  irrecv.enableIRIn();  // Start the receiver

  // Define I/O
  pinMode(S_pin, INPUT);
  pinMode(P_pin, INPUT);
  pinMode(I_pin, INPUT);
  pinMode(D_pin, INPUT);

  for (int i = 0; i < totalPhotoResistors; i++) {
    pinMode(PR_Pins[i], INPUT);
  }

  pinMode(RGB_Pin[0], OUTPUT);
  pinMode(RGB_Pin[1], OUTPUT);
  pinMode(RGB_Pin[2], OUTPUT);

  AFMS.begin();       //for Motor

  /*  Calibrate photoresistors
    Read potentiometers
    Start motors/start moving
  */

  Calibrate();

}

void loop() {
  
  modeDetection();
  delay(100);

  if (mode && mode != 4) { // if mode = 0 the car is effectively off
    
    /*  Read potentiometers
    Read photoresistors
    Calculate error 
    Use PID to calculate turn rate given the error
    Run motors with adjusted turn rate values
    */


    Sp = ReadPotentiometer(S_pin, 0, 1023, 0, 100);
    kP = ReadPotentiometer(P_pin, 0, 1023, 0, 100);
    kI = ReadPotentiometer(I_pin, 0, 1023, 0, 100);
    kD = ReadPotentiometer(D_pin, 0, 1023, 0, 100);

    ReadPhotoResistors();

    CalcError();

    calculatePID();

    // Physical turn.

    RunMotors(PIDvalue);

    Print();
      
    
  } else if (mode == 4) {
     manualControl();
     delay(100);
  }
    else {
    ledOff();
  }
  
}

// Calibration
void Calibrate() {
  writeColor(255,150,0); // YELLOW indicator, lasts for 5 seconds
  delay(5000); // decreased for debugging

  writeColor(105, 105, 105); // GREY indicator means that calibration for black has begun

  int numSamples = 100; // number of samples to calibrate with

  float samples[numSamples]; // stores the samples. Same array can be rewritten for second phase 

  // Calibrate black
  for (int i = 0; i < totalPhotoResistors; i++) {
    for(int j = 0; j < numSamples; j++) {
      samples[j] = analogRead(PR_Pins[i]); // populate samples
    }

    float sum = 0; // take the average and store it accordingly
    for (int k = 0; k < numSamples; k++) {
      sum = sum + samples[k];
    }

    black_values[i] = sum / numSamples;

  }

  delay(1000);


  writeColor(255,150,0); // YELLOW indicator, lasts for 5 seconds
  delay(5000);

  writeColor(255, 255, 255); // WHITE indicator means that calibration for white has begun

  // Calibrate white
  for (int i = 0; i < totalPhotoResistors; i++) {
    for(int j = 0; j < numSamples; j++) {
      samples[j] = analogRead(PR_Pins[i]); // populate samples
    }

    float sum = 0; 
    for (int k = 0; k < numSamples; k++) {
      sum = sum + samples[k];
    }

    white_values[i] = sum / numSamples;

  }

  delay(1000);


}

// Read sensors
void ReadPhotoResistors() {
  for (int i = 0; i < totalPhotoResistors; i++) {
    // Syntax: map(value, fromLow, fromHigh, toLow, toHigh)
    PR_Vals[i] = map(analogRead(PR_Pins[i]), black_values[i], white_values[i], 0, 100);
    delay(10);
  }
}

int ReadPotentiometer(int pin, int min_resolution, int max_resolution, int min_potentiometer, int max_potentiometer) {
  return map(analogRead(pin), min_resolution, max_resolution, min_potentiometer, max_potentiometer);
} // For now i will say this works although it is monkaW

// Calculate Error
void CalcError() {
  // weighted average
  float average = 0;
  int numDetected = 0;
  for (int i = 0; i < totalPhotoResistors; i++) {
    if (PR_Vals[i] < threshold) {
      average = average + (i+1);
      numDetected++;
    }
  }
  average = average / numDetected;
  if (average != 4) {
    error = (4 - (average));
  } else {
    error = 0;
  }


}

void calculatePID() { // I actually have no clue if this is going to work we have to test
  float P = error;
  float I = sumerror;
  float D = error - lastError;
//
  kP = kP / 1; // scale values into 0 - 100
  kI = kI / 1000;
  kD = kD / 100;


  PIDvalue = (kP*P) + (kI*I) +(kD*D);

  sumerror = sumerror + error;
  if (sumerror > 5) {
    sumerror = 5;
  } // prevents integrator wind-up
  else if (sumerror < -5) {
    sumerror = -5;
  }

  lastError = error;

}

// LED Control
void writeColor(int r, int g, int b) {
  Serial.print("LED FUNCTION CALLED");
  analogWrite(RGB_Pin[0], r);
  analogWrite(RGB_Pin[1], b);
  analogWrite(RGB_Pin[2], g);
}

void ledOff() {
  analogWrite(RGB_Pin[0], 0);
  analogWrite(RGB_Pin[1], 0);
  analogWrite(RGB_Pin[2], 0);
}

void colorSwitch() {
  switch (mode) {
    case 0:
      ledOff();
      break;
    case 1:
      writeColor(255, 0, 0);
      break;
    case 2:
      writeColor(0, 255, 0);
      break;
    case 3:
      writeColor(0, 0, 255);
      break;
    case 4:
      writeColor(0,255,255);
    default:
      break;
  }
}

// Read Button
void modeDetection() {
  if (irrecv.decode(&results)) {
    // Serial.println(results.value, HEX);  // Prints HEX code IR receiver receives
    irrecv.resume(); // Receive the next value
    
    // Restart
    if (results.value == ON_OFF_KEY) {
      setup();
    }
    
    if (results.value == A_KEY)      
       mode = 1;
    else if (results.value == B_KEY) 
      mode = 2;
    else if (results.value == C_KEY) // Brightness down
      mode = 3;
    else if (results.value == CIRCLE_KEY) // Circle
      mode = 4;
    
  }
  colorSwitch();
}

void manualControl() {
  if (irrecv.decode(&results)) {
    irrecv.resume();
    if (results.value == UP_ARROW) {
      leftWheel->setSpeed(150);
      leftWheel->setSpeed(150);
      
    } else if (results.value == DOWN_ARROW) { // stop
      leftWheel->setSpeed(0);
      rightWheel->setSpeed(0);
      
    } else if (results.value == RIGHT_ARROW) {
      leftWheel->setSpeed(255);
      rightWheel->setSpeed(0);
      
    } else if (results.value == LEFT_ARROW ) {
      leftWheel->setSpeed(0);
      rightWheel->setSpeed(255);
    }
  }
  leftWheel->run(FORWARD);
  rightWheel->run(FORWARD);
}

// Run the motors
void RunMotors(int turn) {
  if (turn < 0) {
    leftWheel->setSpeed(SpeedM1-abs(turn));
    rightWheel->setSpeed(SpeedM2);
  }
  else if (turn > 0) {
    leftWheel->setSpeed(SpeedM1);
    rightWheel->setSpeed(SpeedM2-abs(turn));
  } else {
    leftWheel->setSpeed(SpeedM1);
    rightWheel->setSpeed(SpeedM2);

  }
  leftWheel->run(FORWARD);
  rightWheel->run(FORWARD);
}

// print stuff to console idk
void Print() {
//  Serial.print("S=");
//  Serial.print(Sp);
//  Serial.print(" P=");
//  Serial.print(kP);
//  Serial.print(" I=");
//  Serial.print(kI);
//  Serial.print(" D=");
//  Serial.println(kD);

//for (int i=0; i<totalPhotoResistors; i++) {
//  Serial.print(PR_Vals[i]);
//  Serial.print(" ");
//}


Serial.print(mode);
Serial.print("\n");


  

  // might have to add print statements depending on debug/testing

}
