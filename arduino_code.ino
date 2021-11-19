
// POTENTIOMETERS FOR PID
const int S_pin = A0; // proportional - analog pin 0
const int P_pin = A1; // proportional - analog pin 1
const int I_pin = A2; // integral - analog pin 2
const int D_pin = A3; // derivative - analog pin 3
const int STATUS_BTN_PIN = 12;
const int RGB_R 10;
const int RGB_G 11;
const int RGB_B 12;
int Sp = 0;           // speed gain coeficient
int kP = 0;           // proportional gain coeficient
int kI = 0;           // integral gain coeficient
int kD = 0;           // derivative gain coeficient
int status_btn = LOW;

#define BLACK 0
#define WHITE 1023
#define totalPhotoResistors 7 // Defining how many photoresistors are used, modify this if more or less are used

//DECLARE FUNCTIONS
void InitWeights(int);
void SmartDelay(int,float);
void ReadPhotoresistors();
void ReadPotentiometers();
void NormalizePhotoresistorValues();
void ReadButtonValues();
//---------------

struct Weight{
  int white_val = WHITE;
  int black_val = BLACK;
};

// Initialize Photo Resistor Array
int PRVals[totalPhotoResistors]; // Since there are 7 photoresistors, we will number them from 0 to 6 (recall array index start with 0)
Weight PRWeights[totalPhotoResistors];

void setup() { /* Setup - runs once (when power is supplied or after reset) */
  Serial.begin(9600); // For serial communication set up
  
  Serial.println("INIT WHITE");
  SmartDelay(3, 0.5);
  InitWeights(WHITE);
  
  Serial.println("INIT BLACK");
  SmartDelay(3, 0.5);
  InitWeights(BLACK);
}

void ReadPhotoresistors() {
  int photoResistorCounter = 0;
  // looping through analog pins A8 to A14 and storing their values into our LDR array
  for (byte pin = A8; pin <= A14; pin++) {
    PRVals[photoResistorCounter] = analogRead(pin);
    photoResistorCounter++;
    delay(2);
  }
}


void InitWeights(int type){
  ReadPhotoresistors();
  if(type==WHITE){//white
    for(int i =0;i<totalPhotoResistors;i++){
      PRWeights[i].white_val = PRVals[i];
    }
  }else if(type==BLACK){//black
    for(int i =0;i<totalPhotoResistors;i++){
      PRWeights[i].black_val = PRVals[i];
    }
  }
}



void NormalizePhotoresistorValues(){
  for(int i =0;i<totalPhotoResistors;i++){
    Weight w = PRWeights[i];
    int val = PRVals[i];
    int outval = map(val, w.white_val, w.black_val, 0, 100);
    PRVals[i] = outval;
  }
}

void ReadPotentiometers(){
  Sp = analogRead(S_pin);
  kP = analogRead(P_pin);
  kI = analogRead(I_pin);
  kD = analogRead(D_pin);

  Sp = map(Sp, 0, 1023, 0, 100);
  kP = map(kP, 0, 1023, 0, 100);
  kI = map(kI, 0, 1023, 0, 100);
  kD = map(kD, 0, 1023, 0, 100);
}

void ReadButtonValues(){
  status_btn = digitalRead(STATUS_BTN_PIN);
}

void loop() { /* Loop - loops forever (until unpowered or reset) */
  
  ReadPhotoresistors();
  ReadPotentiometers();
  NormalizePhotoresistorValues();
  // Call on user-defined function to read Potentiometer values

  //Sp = /* FIX ME, replace this comment with actual function name */ (S_pin, 0, 1023, 0, 100);
  //kP = /* FIX ME, replace this comment with actual function name */ (P_pin, 0, 1023, 0, 100);
  //kI = /* FIX ME, replace this comment with actual function name */ (I_pin, 0, 1023, 0, 100);
  //kD = /* FIX ME, replace this comment with actual function name */ (D_pin, 0, 1023, 0, 100);
  
  //PrintPot(); // Call on user-defined function to print values from potentiometers
  PrintPR();
}

// ************************************************************************************************* //


// ************************************************************************************************* //
// function to print values of interest

void PrintPR(){
  for(int i =0;i<totalPhotoResistors;i++){
    Serial.print("[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(PRVals[i]);
    Serial.println(" ");
  }
}

void PrintPot(){
  Serial.print("S=");
    Serial.print(Sp);
    Serial.print(" P=");
    Serial.print(kP);
    Serial.print(" I=");
    Serial.print(kI);
    Serial.print(" D=");
    Serial.println(kD);
}

void SmartDelay(int n, float m){// {MS}, {TIME INCREMENT (0.5)}
  for(float i =0;i<=n;i+=m){
    delay((int)(m*1000));
    Serial.print("t=");
    Serial.println(n-i);
  }
}