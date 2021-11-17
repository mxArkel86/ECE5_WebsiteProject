
// POTENTIOMETERS FOR PID
const int S_pin = A0; // proportional - analog pin 0
const int P_pin = A1; // proportional - analog pin 1
const int I_pin = A2; // integral - analog pin 2
const int D_pin = A3; // derivative - analog pin 3
int Sp = 0;           // speed gain coeficient
int kP = 0;           // proportional gain coeficient
int kI = 0;           // integral gain coeficient
int kD = 0;           // derivative gain coeficient

//PHOTORESISTOR DEF
#define totalPhotoResistors 7 // Defining how many photoresistors are used, modify this if more or less are used

struct Weight{
  int while_val;
  int black_val;
}
// Initialize Photo Resistor Array
int PRVals[totalPhotoResistors]; // Since there are 7 photoresistors, we will number them from 0 to 6 (recall array index start with 0)
Weight PRWeights[totalPhotoResistors];

void setup() { /* Setup - runs once (when power is supplied or after reset) */
  Serial.begin(9600); // For serial communication set up
}

void ReadPhotoResistors() {
  int photoResistorCounter = 0;
  // looping through analog pins A8 to A14 and storing their values into our LDR array
  for (byte pin = A8; pin <= A14; pin++) {
    PRVals[photoResistorCounter] = analogRead(pin);
    photoResistorCounter++;
    delay(2);
  }
}

void NormalizePhotoResistorValues(){
  for(int i =0;i<totalPhotoResistors;i++){
    Weight w = PRWeights[i];
    int val = PRVals[i];
    int outval = MapPotentiometer(val, w.while_val, w.black_val, 0, 1023);
    PRVals[i] = outval;
  }
}

void loop() { /* Loop - loops forever (until unpowered or reset) */
  
  ReadPhotoResistors();
  NormalizePhotoResistorValues();
  // Call on user-defined function to read Potentiometer values

  //Sp = /* FIX ME, replace this comment with actual function name */ (S_pin, 0, 1023, 0, 100);
  //kP = /* FIX ME, replace this comment with actual function name */ (P_pin, 0, 1023, 0, 100);
  //kI = /* FIX ME, replace this comment with actual function name */ (I_pin, 0, 1023, 0, 100);
  //kD = /* FIX ME, replace this comment with actual function name */ (D_pin, 0, 1023, 0, 100);

  Print(); // Call on user-defined function to print values from potentiometers
}

// ************************************************************************************************* //
// function to read and map values from potentiometers
int MapPhotoResistor(int val, int min_resolution, int max_resolution, int min_potentiometer, int max_potentiometer) {
  return map(val, min_resolution, max_resolution, min_potentiometer, max_potentiometer);
}

// ************************************************************************************************* //
// function to print values of interest
void Print() {

  Serial.print(Sp);
  Serial.print(" ");
  Serial.print(kP);
  Serial.print(" ");
  Serial.print(kI);
  Serial.print(" ");
  Serial.println(kD);

  delay(200); //just here to slow down the output for easier reading if desired
}