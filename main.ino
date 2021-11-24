// TO-DO: ADD LOGIC FOR MODES! I.E., CHANGE CERTAIN FACTORS DEPENDING ON THE CHALLENGE 
// ALSO: FINE TUNE PID CONSTANTS! THIS WILL TAKE TRIAL/ERROR!
// ACTUALLY BOTH OF THESE WILL REQUIRE TESTING!
// ALSO THE FOURTH POTENTIOMETER'S VALUE ISNT REALLY BEING USED (GOING SLOW IS BASED)
// IF ANYONE HAS QUESTIONS ABOUT CODE ASK ME!
// -ARNAV

#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define totalPhotoResistors 7

// Pins
#define P_pin A6
#define I_pin A7
#define D_pin A8
#define buttonPin 41
const int RGB_Pin[3] {49,45,53};
const int PR_Pins[totalPhotoResistors] = {A9, A10, A11, A12, A13, A14,A15};

int PR_Vals[totalPhotoResistors]; // Stores the color of each photoresistor

// Mode that the car is in
int mode = 0; // ranges 0-3

// basespeed
int basespeed = 150;
int maxspeed = 200; // ITS ABOUT SPEED ITS ABOUT POWER 
int motor1speed;
int motor2speed;

int overLine = 1;

// The highest and lowest values for each photoresistor during calibration. 
// Each photoresistor will have different slightly different calibration
float black_values[totalPhotoResistors];
float white_values[totalPhotoResistors];

// Initialize Motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1); // Motors can be switched here (1) <--> (2)Left
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);//Right

// Error
float error, lastError = 0, sumerror = 0;
int threshold = 20; // if photoresistor value is under threshold, it is detecting a line

// PID values
int kP;
int kI;
int kD;



void setup() {
  
  Serial.begin(9600);

  // Define I/O
  pinMode(P_pin, INPUT);
  pinMode(I_pin, INPUT);
  pinMode(D_pin, INPUT);
  pinMode(buttonPin, INPUT);

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
  readButton(); // change mode of car accordingly
  if (mode) { // if mode = 0 the car is effectively off

    /*  Read potentiometers
    Read photoresistors
    Calculate error 
    Use PID to calculate turn rate given the error
    Run motors with adjusted turn rate values
    */

    kP = ReadPotentiometer(P_pin, 0, 1023, 0, 100);
    kI = ReadPotentiometer(I_pin, 0, 1023, 0, 100);
    kD = ReadPotentiometer(D_pin, 0, 1023, 0, 100);

    ReadPhotoResistors();

    CalcError();
    if (overLine) {
      runMotors();
    }

//    CalculateTurn();
//    runMotors();

//    if (overLine) {
//       CalculateTurn(); // PID 
//       runMotors(); // Physical turn. If not over line then 180 in place.
//    } else {
//      motor1speed = basespeed;
//      motor2speed = basespeed;
//      Motor1->run(FORWARD);
//      Motor2->run(BACKWARD);  
//    }
    Print();
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

  ledOff(); // Calibration is complete

}

// Read sensors
void ReadPhotoResistors() {
  for (int i = 0; i < totalPhotoResistors; i++) {
    // Syntax: map(value, fromLow, fromHigh, toLow, toHigh)
    PR_Vals[i] = map(analogRead(PR_Pins[i]), black_values[i], white_values[i], 0, 100);
    if (PR_Vals[i] > 100) {
      PR_Vals[i] = 100;
    }else if (PR_Vals[i] < 0) {
      PR_Vals[i] = 0;
    }
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
  if (numDetected) {
    overLine = 1;
    average = average / numDetected;
    if (average != 4) {
       error = (4 - (average));
    } else {
      error = 0;
    }
  } else {
    overLine = 0;
  }
  


}

void CalculateTurn() { // I actually have no clue if this is going to work we have to test
  float P = error;
  float I = sumerror;
  float D = error - lastError;
//
//  kP = kP / 1; // scale values into 0 - 100
//  kI = kI / 1000;
//  kD = kD / 100;
  int speedChange;
  if (mode != 3) { // extra maybe?
    speedChange = (kP*P) + (kI*I) +(kD*D); // PID turn
  } else {
    speedChange = (kP*P); // proportional turn
  }
  

  sumerror = sumerror + error;
  if (sumerror > 5) {
    sumerror = 5;
  } // prevents integrator wind-up
  else if (sumerror < -5) {
    sumerror = -5;
  }

  lastError = error;

  

  
  int motor1speed = basespeed + speedChange; // MIGHT HAVE TO SWITCH MOTORS 
  int motor2speed = basespeed - speedChange;
  
  if (motor1speed > maxspeed) {
    motor1speed = maxspeed;
  }
  if (motor2speed > maxspeed) {
    motor2speed = maxspeed;
  }
  if (motor1speed < 0) {
    motor1speed = 0;
  }
  if (motor2speed < 0) {
    motor2speed = 0;
  } 

}

// LED Control
void writeColor(int r, int g, int b) {
  analogWrite(RGB_Pin[0], r);
  analogWrite(RGB_Pin[1], b);
  analogWrite(RGB_Pin[2], g);
}

void ledOff() {
  analogWrite(RGB_Pin[0], 0);
  analogWrite(RGB_Pin[1], 0);
  analogWrite(RGB_Pin[2], 0);
}

void Switch() { 
  switch (mode) {
    case 0: // off
      ledOff();
      basespeed = 0;
      maxspeed = 0;
      break;
    case 1: // circle
      writeColor(255, 0, 0);
      basespeed = 150;
      maxspeed = 200;
      break;
    case 2: // frequency sweep
      writeColor(0, 255, 0);
      basespeed = 100;
      maxspeed = 150;
      break;
    case 3: // drag race
      writeColor(0, 0, 255);
      basespeed = 200;
      maxspeed = 255;
      break;
    default:
      break;
  }
}

// Read Button
void readButton() {
  int buttonState = digitalRead(buttonPin);
  if(buttonState == HIGH) {
    if (mode < 3) {
      mode++;
    } else {
      mode = 0;
    }
    delay(200);
  } 
  
  Switch();
}

// Run the motors
void runMotors() {
  Motor1->setSpeed(motor1speed);
  Motor2->setSpeed(motor2speed);

  Motor1->run(FORWARD);
  Motor2->run(FORWARD);
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


//
//



for (int i = 0; i < totalPhotoResistors; i++) {
  Serial.print(analogRead(PR_Pins[i]));
  //Serial.print(PR_Vals[i]);
  Serial.print(" ");
}
Serial.print("\n");


  

  // might have to add print statements depending on debug/testing

}
