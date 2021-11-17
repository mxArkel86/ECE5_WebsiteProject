/* Calibration Code UCSD ECE 5 Lab 4 */
// Variables and Libaries for Motor

#define totalPhotoResistors 7 // Defining how many photoresistors are used, modify this if more or less are used

// Variables for Light Sensors
int LDR_Pin[totalPhotoResistors] = {A8, A9, A10, A11, A12, A13, A14}; // Arrays are used top simplify the code
int LDR[totalPhotoResistors];                                         // these are variables that have multiple elements to each variable name
                                                                      // LDR_Pin hold 7 values and A8 is the 0th element and A11 is the 4th element
                                                                      // Calibration Variables
int led_Pin = 13;                                                     /// This pin is for a led built into the Arduino that indicates what part of the calibration you are on
                                                                      /// You can use any digital pin like digital pin 31 with an LED connected for better visibility
float Mn[totalPhotoResistors];
float Mx[totalPhotoResistors];
float LDRf[totalPhotoResistors] = {0., 0., 0., 0., 0., 0., 0.};

float AveRead;
int MaxReading;
int MaxReadingPinNumber;
int CriteriaForMax;
float WeightedAve;
float error;

// ************************************************************************************************* //
// setup - runs once
void setup() {

  Serial.begin(9600);       // For serial communication set up
  pinMode(led_Pin, OUTPUT); // Note that all analog pins used are INPUTs by default so don't need pinMode

  Calibrate(); // Calibrate black and white sensing

} // end setup()

// ************************************************************************************************* //
// loop - runs/loops forever
void loop() {

  ReadPhotoResistors(); // Read photoresistors and map to 0-100 based on calibration

  CalcError(); //Calculate ther error of photoresistor reads

  Print(); // Print values to serial monitor  //currently commented out but could be good for debugging =)

} // end loop()

// ************************************************************************************************* //
/**
 * function to calibrate
 */
void Calibrate() {
  // Calibration
  int numberOfMeasurements = 40;
  // White Calibration
  CalibrateHelper(numberOfMeasurements, false);

  // Time to move from White to Black Surface
  // Black Calibration
  CalibrateHelper(numberOfMeasurements, true);

} // end Calibrate()

/**
 * Helper function to help with calibration of black and white measurements
 *
 * @param numberOfMeasurements: the pin of the potentiometer we want to read from
 * @param ifCalibratingBlack: a booleaning indicating if you are testing for black or bright
 */
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
    if (ifCalibratingBlack) { // updating cooresponding arry based on if we are calibrating black or white
      Mx[pin] = round(LDRf[pin] / (float)numberOfMeasurements); // take average
    }
    else {
      Mn[pin] = round(LDRf[pin] / (float)numberOfMeasurements); // take average
    }
    LDRf[pin] = 0.0;
  }
}

// ************************************************************************************************* //
/**
 * function to read photo resistors, map from 0 to 100, and find darkest photo resitor (MaxReadingPinNumber).
 */
void ReadPhotoResistors() {
  for (int Li = 0; Li < totalPhotoResistors; Li++) {
    LDR[Li] = map(analogRead(LDR_Pin[Li]), Mn[Li], Mx[Li], 0, 100);
    delay(2);
  }
} // end ReadPhotoResistors()

// ************************************************************************************************* //
/**
 * Calculate error from photoresistor readings
 */
void CalcError() {
  AveRead = 0.0;  // reset the average photoresistor read variable

  // finding the photoresistor that sees the darkest
  int MaxReadingIndex = getDarkestIndex(); // this stores the LDR array index of the darkest value 
  float MaxReading = LDR[MaxReadingIndex];     // this stores the value of darkest photoresistor

  CriteriaForMax = 2; // maxReading should be at least twice as big as the other values (if the max reading isn't twice as big as AveRead, we don't use it)
  if (MaxReading > CriteriaForMax * AveRead) {
    // get the index of the photoresistor on the left and right
    int leftIndex = MaxReadingIndex - 1;
    int rightIndex = MaxReadingIndex + 1;

    // verify the photoresistor indexes are in range
    if (leftIndex < 0) { 
      leftIndex = 0;
    }
    if (rightIndex > totalPhotoResistors - 1) {
      rightIndex = totalPhotoResistors - 1;
    }

    // we now attempt to calulate the weighted average
    float numerator, denominator = 0.0;
    for (int i = leftIndex; i <= rightIndex; i++) {
      numerator = numerator + LDR[i] * i;
      denominator = denominator + LDR[i];
    }

    // make sure we don't have a divide by zero error
    if (denominator != 0.0){
      return;
    }

    // calculate and store the weighted average
    WeightedAve = (float)numerator/(float)denominator;
    error = -1 * (WeightedAve - 3); // we set the error variable from our calculation
  }
} // end CalcError()

/**
 * finding the photoresistor that sees the darkest, also updating AveRead to the newest average photoresistor reading
 *
 * @return the LDR array index of the photoresistor that sees the darkest area 
 */
int getDarkestIndex(){
  int maxIndex = -1;
  float max = -99.9;
  for (int i = 0; i < totalPhotoResistors; i++) {
    if (max < LDR[i]) {
      max = LDR[i];
      maxIndex = i;
    }
    AveRead = AveRead + (float)LDR[i] / (float)totalPhotoResistors;
  }

  // figuring out which pin has the darkest value
  MaxReadingPinNumber = -1 * (maxIndex - 3);
  return maxIndex;
}

// ************************************************************************************************* //
/**
 * Function to print values of interest
 */void Print() {
  for (int i = 0; i < totalPhotoResistors; i++) { // Each photo resistor value is shown
    // Printing the photo resistor reading values one by one
    Serial.print(LDR[i]);
    Serial.print(" ");
  }
  Serial.print("   "); // This just prints a gap

  Serial.print(MaxReading);
  Serial.print(" "); // the maximum value from the photo resistors is shown again
  Serial.print(MaxReadingPinNumber);
  Serial.print("    "); // this is the index of that maximum (0 through 6) (aka which element in LDR)
  Serial.print(error);
  Serial.println("    "); // this will show the calculated error (-3 through 3)

  delay(100); //just here to slow down the output for easier reading if wanted

} // end Print()
