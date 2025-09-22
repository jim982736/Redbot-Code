#include <Servo.h>
#include <math.h>

#define AVERAGE(a, b) ((a + b) / 2)

// Left Motor
#define AIN1 13
#define AIN2 12
#define APWR 11

#define L_ENC_A 3   // White
#define L_ENC_B A0  // Green

// Right Motor
#define BIN1 8
#define BIN2 9
#define BPWR 10

#define R_ENC_A 2   // White
#define R_ENC_B A1  // Green
// Blue -> GND
// Yellow -> 5V

const int servoPin = 6;
const int buttonPin = 7;

volatile int rightEncoderValue = 0;
volatile int leftEncoderValue = 0;

// unused variables meant for debugging
volatile int upCnt = 0;
volatile int dwnCnt = 0;

// defining as bytes since these are read directly from ports
byte rStateA = 0;
byte rStateB = 0;
byte lStateA = 0;
byte lStateB = 0;

const int countsPerRev = 288;  // 48:1 gear ratio, 6 ticks per revolution on encoder
const double wheelDia = 6.6;   // measured wheel to be about 66mm in diameter
const double wheelCirc = PI * wheelDia;
const double wheelDist = 5.0;  // approx distance from the center of the wheels to the center of the robot

// drive distance cm at power adjusting the motors every rate ms
void driveStraight(double distance, int power, int rate = 50);

int lCnt = 0;
int rCnt = 0;

int logl[250] = {0};
int logr[250] = {0};

void setup() {
  logl[499] = 1000000;
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(APWR, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BPWR, OUTPUT);

  pinMode(L_ENC_A, INPUT);
  pinMode(L_ENC_B, INPUT);
  pinMode(R_ENC_A, INPUT);
  pinMode(R_ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, CHANGE);

  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(9600);
  //clearEncoders();
}

void loop() {
  /*int left = 0, right = 0;
  readEncoders(&left, &right);
  Serial.print(left);
  Serial.print('\t');
  Serial.println(right);
  delay(50);*/

  if (digitalRead(buttonPin) == LOW) {
    Serial.println("Go");
    //driveStraight(30.0, 200);

    clearEncoders();
    reset();

    int lCount = 0;
    int rCount = 0;

    int* leftLog = logl;
    int* rightLog = logr;

    int errorCount = 0;

    driveForward(200);
    readEncoders(&lCount, &rCount);
    while (AVERAGE(lCount, rCount) < 1000) {
      readEncoders(&lCount, &rCount);

      if (*leftLog != 1000000) {
        *leftLog = lCount;
        *rightLog = rCount;
        leftLog++;
        rightLog++;
      }

      if (lCount < 0 || rCount < 0) {
        errorCount++;
      }

      delay(10);
    }
    brake();
    Serial.println("done");

    // print logs out
    leftLog = logl;
    rightLog = logr;

    Serial.println("Left   Right   Target");
    for (int i = 0; i < 250; i++) {
      Serial.print(*leftLog++);
      Serial.print("\t");
      Serial.print(*rightLog++);
      Serial.print("\t");
      Serial.println(500);
    }

    noInterrupts();
    int up = upCnt;
    int down = dwnCnt;
    interrupts();

    Serial.print("Error Count: ");
    Serial.println(errorCount);
    Serial.print("up: ");
    Serial.print(up);
    Serial.print("\tdown: ");
    Serial.println(down);
  } 

  /*if (digitalRead(buttonPin) == LOW) {
    readEncoders(&lCnt, &rCnt);
    Serial.print(lCnt);
    Serial.print("\t");
    Serial.println(rCnt);
  } else {
    clearEncoders();
  }
  delay(50);*/ 
}

void reset() {
  for (int i = 0; i < 250; i++) {
    logl[i] = logr[i] = 0;
  }

  noInterrupts();
  upCnt = 0;
  dwnCnt = 0;
  interrupts();
}

// Drive Control
void driveStraight(double distance, int power, int rate) {
  delay(100);

  int lCount = 0;
  int rCount = 0;
  int targetCount;
  double revCount;
  double overshoot = 0.0;  // measure overshoot

  int lPrevCount = 0, rPrevCount = 0;  // variables to track left and right encoder counts
  int lDiff, rDiff;                    // difference between current and previous encoder counts

  int leftPower = power, rightPower = power;  // variables for adjusting motor power

  int offset = 5;
  double error;

  revCount = (distance - overshoot) / wheelCirc;  // target number of rotations, subtracting overshoot
  targetCount = revCount * countsPerRev;

  // print header
  Serial.print("driveStraight target: ");
  Serial.print(targetCount);
  Serial.println(" revolutions");
  Serial.println("Left   Right   Target   Error");

  clearEncoders();
  delay(200);  // short delay before starting motors

  driveForward(power);

  long cnt = 0;
  int time = 0;
  int startTime = millis();

  while (AVERAGE(lCount, rCount) < targetCount) {
    // Wait until reaching target while adjusting encoders to keep line straight

    if (time >= 50) {
      time = 0;
      startTime = millis();

      readEncoders(&lCount, &rCount);

      Serial.print(lCount);
      Serial.print("\t");
      Serial.print(rCount);
      Serial.print("\t");
      Serial.print(targetCount);
      Serial.print("\t");

      // account for some occasional sporadic readings at the beginning of driveStraight
      /*if (cnt < 3 && (lCount < 0 || rCount < 0)) {
        Serial.println("\n---reset---\n");
        clearEncoders();
        lCount = rCount = 0;
      }
      cnt++;*/

      // calculate difference in encoder counts from previous cycle
      lDiff = (lCount - lPrevCount);
      rDiff = (rCount - rPrevCount);
      error = (double)abs((lDiff - rDiff)) / 5.0;  // determine error between encoder counts and normalize it by 5
      Serial.println(error);

      // store the current count as the "previous" count for the next cycle.
      lPrevCount = lCount;
      rPrevCount = rCount;

      // skip update if error seems to large, likely innacurate
      /*if (error > 3.0) {
        delay(rate);
        continue;
      }*/

      // if left is faster than the right, slow down the left / speed up right
      if (lDiff > rDiff) {
        leftPower -= (int)(offset * error);
        rightPower += (int)(offset * error);
      }
      // if right is faster than the left, speed up the left / slow down right
      else if (lDiff < rDiff) {
        leftPower += (int)(offset * error);
        rightPower -= (int)(offset * error);
      }

      // update motor power
      driveLeft(leftPower);
      driveRight(rightPower);
    }
    time = millis() - startTime;
  }
  // stop after exiting loop
  brake();
}

// Motor Control Functions
void driveRight(int power) {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(BPWR, power);
}

void driveLeft(int power) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(APWR, power);
}

void driveForward(int power) {
  driveLeft(power);
  driveRight(power);
}

void coast() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void brake() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(APWR, 0);
  analogWrite(BPWR, 0);
}

// Encoder Control
void clearEncoders() {
  noInterrupts();
  rightEncoderValue = 0;
  leftEncoderValue = 0;
  interrupts();
}

void readEncoders(int* left, int* right) {
  noInterrupts();
  *left = leftEncoderValue;
  *right = rightEncoderValue;
  interrupts();
}

// Encoder ISRs
// Reading directly from ports to speed up runtime
void leftEncoderISR() {
  lStateA = (PIND & B00001000) / 8;  // pin 3 on port D
  lStateB = PINC & B00000001;        // pin A0 on port C
  // dividing to make sure states are between 0 and 1

  /*lStateA = digitalRead(L_ENC_A);
  lStateB = digitalRead(L_ENC_B);*/

  if (lStateA == lStateB) {
    leftEncoderValue--;
  } else {
    leftEncoderValue++;
  }
}

void rightEncoderISR() {
  rStateA = (PIND & B00000100) / 4;  // pin 2 on port D
  rStateB = (PINC & B00000010) / 2;  // pin A1 on port C
  // dividing to make sure states are between 0 and 1

  /*rStateA = digitalRead(R_ENC_A);
  rStateB = digitalRead(R_ENC_B);*/

  if (rStateA == rStateB) {
    rightEncoderValue++;
  } else {
    rightEncoderValue--;
  }
}
