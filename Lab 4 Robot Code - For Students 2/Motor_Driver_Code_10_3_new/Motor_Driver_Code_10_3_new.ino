/* Motor Driver Code UCSD ECE 5 Lab 4*/
// Libraries for Motor
#include <Wire.h>
#include <Adafruit_MotorShield.h> // Must add libary - see MotorShield Manual
//https://cdn-learn.adafruit.com/downloads/pdf/adafruit-motor-shield-v2-for-arduino.pdf

// Initialize Motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1); // Motors can be switched here (1) <--> (2)
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);

//Set Initial Speed of Motors
int M1Sp = 60; // initial speed may vary and later can be increased with Sp potentiometer
int M2Sp = 60;

//Set LED Pin
int led_Pin = 13; // can change to another digital pin and connect extra LED to me more easily seen

// A function that commands a specified motor to move towards a given direction at a given speed
void runMotorAtSpeed(Adafruit_DCMotor *motor, int speed, int direction) {
   motor->setSpeed(speed); // sets the speed of the motor from arguments
   motor->run(direction);  // sets the direction of the motor from arguments
}

// setup - runs once
void setup() {
   Serial.begin(9600); // For serial communication set up
   AFMS.begin();       //for Motor

   pinMode(led_Pin, OUTPUT); // set pin mode to output voltage
                             // Gives you a moment before tank actually moves
   for (int i = 0; i < 20; i++) {
      digitalWrite(led_Pin, HIGH); // turn the LED on (HIGH is the voltage level)
      delay(100);                  // wait for 100 milliseconds
      digitalWrite(led_Pin, LOW);  // turn the LED off by making the voltage LOW
      delay(100);                  // wait for 100 milliseconds
   }
}

// loop - loops forever
void loop() {

   // Start Motors in forward direction
   runMotorAtSpeed(Motor1, M1Sp, FORWARD);
   runMotorAtSpeed(Motor2, M2Sp, FORWARD);
   delay(3000); // let run forward for 3 seconds

   // Start Motors in backward direction
   /** FIX ME: set motor 1 at M1Sp in the BACKWARD direction (HINT: Very similar to forward direction) **/
   /** FIX ME: set motor 2 at M2Sp in the BACKWARD direction **/
   delay(3000); // let run backward for 3 seconds

   // Stop Motors
   /** FIX ME: stop motor 1 by using RELEASE as direction, use M1Sp for speed here **/
   /** FIX ME: stop motor 2 by using RELEASE as direction, use M2Sp for speed here **/
   delay(3000); // stop for 3 seconds
}
