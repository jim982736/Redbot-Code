// libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_AS726x.h"

#include <utility/imumaths.h>
#include <Vector.h>
#include <string.h>

#include <RedBot.h>
#include <Servo.h>

// additional helper functions
#include "helper.h"
#include "pid.h"

// for use with driveStraight, generally use SLOW for shorter
// distances (< 20cm) and FAST for longer distances
#define SLOW 50
#define FAST 100

// Create objects
RedBotMotors motors;
Adafruit_AS726x ams;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
RedBotEncoder encoder = RedBotEncoder(A2, 10);

// useful for testing imu
uint8_t stat_flag = 5;
uint8_t err_flag = 0;
uint8_t calib_flag = 0B11111100;

int buttonPin = 12;

int indicatorLED = 3;

const int servoPin = 9;
Servo marker;

void dropMarker() {
  marker.write(45);
}
void raiseMarker() {
  marker.write(0);
}

// Vector for storing absolue position information
double pose[3] = { 0 };  // x, y, theta, x and y in encoder ticks, theta in radians
imu::Quaternion ref;     // used to track starting orientation
// double xLocation = 0.0;  // in encoder ticks
// double yLocation = 0.0;  // in encoder ticks
// double orientation = 0.0;

// Signature variables
float calibratedValues[AS726x_NUM_CHANNELS];
double sensorLevels[AS726x_NUM_CHANNELS];
String colorNames[3] = { "Unknown", "Red", "Blue" };
Signature Red;
Signature Blue;

// Uses IMU and encoders to attempt to accurately determine actual location and stay on track better,
// x and y in inches
void driveToPoint(double x, double y, int motorPower, bool draw = true);

// speed controller used by turnToAnglePID()
int SCStorage[2];
Vector<int> SpeedController(int drive, int lPrev, int rPrev);

// controller used to maintain straight path
void UpdateMotorPower(Vector<double>& targetPath, Vector<double>& path, int& leftPower, int& rightPower, bool reverse, double dev, int basePower);

// Update pose by using data from the color sensor, motor encoders, and IMU
void EvaluatePose(uint8_t color, bool clear = false);
void UpdatePose(int x, int y);

bool start = false;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(indicatorLED, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A0, INPUT);
  randomSeed(analogRead(A0));

  marker.write(0);
  marker.attach(servoPin);
  Serial.begin(115200);

  flashLED(LED_BUILTIN, 4, 100);
  flashLED(indicatorLED, 3, 200);

  // Make sure the sensors are connected properly and working
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  bno.setExtCrystalUse(true);
  // IMUPLUS uses only accelerometer and gyroscope with sensor fusion,
  // not using magnetometer due to calibration issues
  // bno.setMode(OPERATION_MODE_IMUPLUS);

  if (!ams.begin()) {
    Serial.println("could not connect to sensor! Please check your wiring.");
    while (1)
      ;
  }

  bool led = false;

  Serial.println("Beginning IMU calibration");
  while (!bno.isFullyCalibrated()) {
    if (led) {
      digitalWrite(LED_BUILTIN, LOW);
      led = false;
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
      led = true;
    }
    delay(250);
  }
  Serial.println("Finished Calibration\n");
  flashLED(LED_BUILTIN, 2, 100);

  // Set Starting Point
  flashLED(LED_BUILTIN, 8, 500);
  ref = bno.getQuat();

  // Room lights, no LED, mounted on underside, using red - #ff3131 and blue - #0e4996
  // Based on Calibrated readings
  double redLevels[] = { 0.0388, 0.0577, 0.1033, 0.1602, 0.3257, 0.3144 };
  double redStdDev[] = { 0.0032, 0.0118, 0.0602, 0.0623, 0.0615, 0.0716 };
  double redAvgTotal = 468.617;

  double blueLevels[] = { 0.1402, 0.1418, 0.1747, 0.1732, 0.2342, 0.1359 };
  double blueStdDev[] = { 0.0149, 0.0306, 0.0148, 0.0155, 0.0336, 0.0134 };
  double blueAvgTotal = 138.189;

  PopulateSignature(Red, redLevels, redStdDev, redAvgTotal);
  PopulateSignature(Blue, blueLevels, blueStdDev, blueAvgTotal);

  pose[2] = (bno.getQuat() * ref.conjugate()).toEuler().x();
  Serial.print("Current Orientation: ");
  Serial.println(pose[2]);
  Serial.println("Ready");
}

void loop(void) {
  // set the power for left & right motors on button press
  if (digitalRead(buttonPin) == LOW) {
    RandomBuild2(5.0, 5, 7);
    // marker.write(45);
    // delay(1000);
    // marker.write(0);
    // delay(1000);

    // driveToPoint(15.0, 0, SLOW, false);
    // driveToPoint(15.0, 5.0, SLOW, false);

    // driveToPoint(30.0, SLOW, false);
    // turnToAnglePID(120.0);
    // driveToPoint(15.0, SLOW, false);
    // turnToAnglePID(-120.0);
    // driveToPoint(30.0, SLOW, false);
    // turnToAnglePID(120.0);
    // driveToPoint(15.0, SLOW, false);

    // Zigzag with 90degree angles
    /*driveToPoint(15.0, FAST, false);
    turnToAnglePID(135.0);
    driveToPoint(21.2, FAST);
    marker.write(15);*/

    // start = true;
  }
  if (start) {
    RandomBuild2(5.0, 5, 7);
  }
}

void checkErr() {
  // Check SYS_STAT register
  Wire.beginTransmission(0x28);
  Wire.write(bno.BNO055_SYS_STAT_ADDR);
  Wire.endTransmission();
  Wire.requestFrom(0x28, 1);
  uint8_t stat = Wire.read();

  // Check SYS_ERR register
  Wire.beginTransmission(0x28);
  Wire.write(bno.BNO055_SYS_ERR_ADDR);
  Wire.endTransmission();
  Wire.requestFrom(0x28, 1);
  uint8_t err = Wire.read();

  // Check CALIB_STAT
  Wire.beginTransmission(0x28);
  Wire.write(bno.BNO055_CALIB_STAT_ADDR);
  Wire.endTransmission();
  Wire.requestFrom(0x28, 1);
  uint8_t calib = Wire.read();

  if (stat != 5) {
    stat_flag = stat;
  }
  if (err != 0) {
    err_flag = err;
  }
  if (calib == 0) {
    calib_flag = calib;
  }
}

void printCheck() {
  Serial.print(stat_flag);
  Serial.print(", ");
  Serial.print(err_flag, HEX);
  Serial.print(", ");
  Serial.println(calib_flag, BIN);
}

// Turn to absolute position between -180 and 180 degrees
double turnToAnglePID(double angle) {
  if (angle < -180.0 || angle > 180.0) {
    return -1.0;
  }

  raiseMarker();

  pose[2] = (bno.getQuat() * ref.conjugate()).toEuler().x();

  // double target = pose[2] + RAD(angle);
  double target = RAD(angle);

  if (target > PI) {
    target = target - 2.0 * PI;
  } else if (target < (PI * -1.0)) {
    target = target + 2.0 * PI;
  }

  double dist = 0.0;
  double startOrientation = pose[2];

  // Serial.print("Target: ");
  // Serial.println(target);

  // control variables
  double error = target;
  int drive = 0;
  int ceiling = 80;  // highest motor power with ability to track movement
  int finishCount = 0;

  int driveStor[2];
  Vector<int> drives(driveStor);
  drives.push_back(drive);
  drives.push_back(drive);

  // variables for storing encoder data
  long lCount = 0, rCount = 0;
  long sL = 0, sR = 0;  // distance encoders moved between samples
  double s = 0.0;
  int lPrev = 0;
  int rPrev = 0;

  int sampleRate = 50;
  int sampleTime = (1.0 / ((double)sampleRate)) * 1000;  // time between samples, in ms
  unsigned int time = 0;                                 // used to track amount of time passed each loop iteration
  int startTime = 0;

  // initialize PID controller for turning, adjust gains to tune controller
  PID pid;
  // old settings
  // pid.propGain = 625;
  // pid.integGain = 31;
  // pid.derGain = 1250;
  // pid.derState = 0.0;
  // pid.integMax = 20.0;
  // pid.integMin = -20.0;
  // pid.integState = 0.0;

  // new settings
  pid.propGain = 225;
  pid.integGain = 3;
  pid.derGain = 937;
  pid.derState = 0.0;
  pid.integMax = 20.0;
  pid.integMin = -20.0;
  pid.integState = 0.0;

  // Serial.println("Error\tDist\tPose\tDrive");

  encoder.clearEnc(BOTH);
  delay(200);
  motors.pivot(ceiling * -1);
  startTime = millis();

  while (finishCount < 10) {
    if (time >= sampleTime) {
      startTime = millis();
      lCount = encoder.getTicks(LEFT);
      rCount = encoder.getTicks(RIGHT);

      // sensors_event_t event;
      // bno.getEvent(&event);

      imu::Quaternion quat = bno.getQuat();
      imu::Vector<3> euler = (quat * ref.conjugate()).toEuler();

      //PID controller to determine motor power
      // dist = euler.x() - startOrientation;
      dist = atan2(sin(euler.x() - startOrientation), cos(euler.x() - startOrientation));

      error = atan2(sin(target - euler.x()), cos(target - euler.x()));
      drive = updatePID(&pid, error, dist);

      // Use to keep track of "walking" behavior while turning
      sL = lCount - lPrev;
      sR = rCount - rPrev;
      s = AVERAGE(sL, sR);
      pose[0] += s * cos(pose[2]);
      pose[1] += s * sin(pose[2]);

      // give drive a ceiling to ensure accurate readings
      drive = constrain(drive, -1 * ceiling, ceiling);

      // hold for 10 samples to end turn
      if (ABS(error) < 0.02) {
        finishCount++;
      } else {
        finishCount = 0;
      }

      drives = SpeedController(drive, lPrev, rPrev);

      // Experimental results suggests that the left motor has a bit less torque
      // especially when driven at very low speeds
      if (ABS(drive) < 30) {
        drives[0] *= 1.6;
      }

      if (drive > 0) {
        motors.leftDrive(drives[0] * -1);
        motors.rightDrive(drives[1]);
      } else if (drive < 0) {
        motors.leftDrive(drives[0]);
        motors.rightDrive(drives[1] * -1);
      } else {
        motors.pivot(drive * -1);
      }

      lPrev = lCount;
      rPrev = rCount;
      pose[2] = euler.x();

      // Serial.print(error, 3);
      // Serial.print('\t');
      // Serial.print(dist, 2);
      // Serial.print('\t');
      // Serial.print(pose[2], 2);
      // Serial.print('\t');
      // Serial.print(drive);
      // Serial.print('\t');
      // printCheck();
    }

    time = millis() - startTime;
    checkErr();
  }
  motors.brake();
  delay(500);

  pose[2] = (bno.getQuat() * ref.conjugate()).toEuler().x();

  return 0.0;
}

// Used to try to match the speeds of the motors while turning to minimize "walking" behavior
Vector<int> SpeedController(int drive, int lPrev, int rPrev) {
  Vector<int> drives(SCStorage);
  drives.push_back(ABS(drive));
  drives.push_back(ABS(drive));

  double propGain = 0.15;
  int lDiff = encoder.getTicks(LEFT) - lPrev;
  int rDiff = encoder.getTicks(RIGHT) - rPrev;
  lDiff = ABS(lDiff);
  rDiff = ABS(rDiff);

  int error = lDiff - rDiff;
  if (error > 0) {
    drives[0] -= error * (propGain / 2.0);
    drives[1] += error * propGain;
  } else if (error < 0) {
    drives[0] += error * propGain;
    drives[1] -= error * (propGain / 2.0);
  }

  // Serial.print(drives[0]);
  // Serial.print("\t");
  // Serial.print(drives[1]);
  // Serial.print("\t");
  // Serial.println(error);

  return drives;
}

// Uses IMU and encoders to attempt to accurately determine actual location and stay on track better.
// x and y in inches
void driveToPoint(double x, double y, int motorPower, bool draw = true) {
  // variables for handling color sensor
  bool rdy = false;
  static bool waiting = false;
  double total = 0.0;
  double redScore = 0.0, blueScore = 0.0;
  uint8_t color = 0;  // 0 = unknown, 1 = red, 2 = blue

  // variables for storing encoder data
  long lCount = 0, rCount = 0;
  long prevlCount = 0, prevrCount = 0;
  long sL = 0, sR = 0;  // distance encoders moved between samples
  double s = 0.0;

  // Start point
  double startX = pose[0];
  double startY = pose[1];
  pose[2] = (bno.getQuat() * ref.conjugate()).toEuler().x();

  // Variables for managing path tracking
  double targetX = ((x * 2.54) / wheelCirc) * countsPerRev;
  double targetY = ((y * 2.54) / wheelCirc) * countsPerRev;
  double TPstorage[2];
  Vector<double> targetPath(TPstorage);  // vector that points from the start point to the target point
  targetPath.push_back(targetX - startX);
  targetPath.push_back(targetY - startY);

  // determining total straight distance we need to travel
  double targetCount = Norm(targetPath);

  double Pstorage[2];
  Vector<double> path(Pstorage);  // vector that points to current position from the start point
  path.push_back(0);
  path.push_back(0);

  // Determining if we should turn first
  double currDirStor[2];
  Vector<double> currDir(currDirStor);
  currDir.push_back(cos(pose[2]));
  currDir.push_back(sin(pose[2]));

  bool reverse = false;
  double theta = acos(Dot(currDir, targetPath) / (Norm(targetPath)));

  if (theta > RAD(5.0) && theta < RAD(175.0)) {
    if (Cross(currDir, targetPath) > 0.0) {
      // turn CCW
      double newOrientation = atan2(sin(pose[2] + theta), cos(pose[2] + theta));
      turnToAnglePID(DEG(newOrientation));
    } else {
      // turn CW
      double newOrientation = atan2(sin(pose[2] - theta), cos(pose[2] - theta));
      turnToAnglePID(DEG(newOrientation));
    }
    startX = pose[0];
    startY = pose[1];
    targetPath[0] = targetX - startX;
    targetPath[1] = targetY - startY;

  } else if (theta > RAD(175.0)) {
    reverse = true;
  }

  // variables for handling motor power
  int leftPower = motorPower;
  int rightPower = motorPower;

  // sample rate used by controller, in Hz
  int sampleRate = 20;
  int sampleTime = (1.0 / ((double)sampleRate)) * 1000;  // time between samples, in ms
  int time = 0;                                          // used to track amount of time passed each loop iteration
  int startTime = millis();

  // double devStorage[2] = { 0.0, 0.0 };
  // Vector<double> deviation(devStorage);
  // deviation.push_back(0.0);
  // deviation.push_back(0.0);
  double dev = 0.0;                 // distance robot has deviated from a straight line
  double dist = 0.0;                // straight distance travelled by robot
  double targetDist = targetCount;  // distance from target position

  bool breaking = false;
  int breakCnt = 0;

  // debug
  // Serial.print("Start Point: ");
  // Serial.print(startX);
  // Serial.print(", ");
  // Serial.print(startY);
  // Serial.print(", ");
  // Serial.println(pose[2]);
  // Serial.print("Target Point: ");
  // Serial.print(targetX);
  // Serial.print(", ");
  // Serial.print(targetY);
  // Serial.print(", ");
  // Serial.println(pose[2]);
  //Serial.println("\nLeft\tRight\tTarget\ts\torientation\tdev\tdist\tlPow\trPow\tpath x\tpath y\tcolor");
  Serial.println("\ns\torientation\tdev\tdist\tlPow\trPow\tpath x\tpath y\tcolor");

  if (draw) {
    dropMarker();
  } else {
    raiseMarker();
  }

  encoder.clearEnc(BOTH);
  delay(200);  // short delay before starting motors
  if (!reverse) {
    motors.drive(motorPower);
  } else {
    motors.drive(-1 * motorPower);
  }

  while (breakCnt < 10) {
    lCount = encoder.getTicks(LEFT);
    rCount = encoder.getTicks(RIGHT);

    if (breaking) {
      breakCnt++;
    }

    // Color sensor handling
    if (!waiting) {
      ams.startMeasurement();  //begin a measurement
      waiting = true;
    }
    rdy = ams.dataReady();

    if (rdy) {
      ams.readCalibratedValues(calibratedValues, AS726x_NUM_CHANNELS);
      waiting = false;

      total = sum(6, calibratedValues);

      sensorLevels[AS726x_VIOLET] = (double)calibratedValues[AS726x_VIOLET] / total;
      sensorLevels[AS726x_BLUE] = (double)calibratedValues[AS726x_BLUE] / total;
      sensorLevels[AS726x_GREEN] = (double)calibratedValues[AS726x_GREEN] / total;
      sensorLevels[AS726x_YELLOW] = (double)calibratedValues[AS726x_YELLOW] / total;
      sensorLevels[AS726x_ORANGE] = (double)calibratedValues[AS726x_ORANGE] / total;
      sensorLevels[AS726x_RED] = (double)calibratedValues[AS726x_RED] / total;

      redScore = Score(Red, total, sensorLevels);
      blueScore = Score(Blue, total, sensorLevels);

      if (redScore > blueScore) {
        color = 1;
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        color = 2;
        digitalWrite(LED_BUILTIN, LOW);
      }

      // Update position based on color sensor data
      if (!CheckPose(pose, color)) {
        // Disagreement between color sensor reading and current pose
        digitalWrite(indicatorLED, HIGH);
        EvaluatePose(color);
      } else {
        // Color sensor and pose agree
        digitalWrite(indicatorLED, LOW);
        // EvaluatePose(color, true);
      }
    }

    // If the robot passed the target point start breaking
    if (dist > (targetCount)) {
      motors.brake();
      breaking = true;
    }

    // If the robot is sufficiently close start breaking
    if (targetDist < 10) {
      motors.brake();
      breaking = true;
    }

    if (time >= sampleTime) {
      startTime = millis();

      imu::Vector<3> euler = (bno.getQuat() * ref.conjugate()).toEuler();
      pose[2] = euler.x();

      sL = lCount - prevlCount;
      sR = rCount - prevrCount;
      s = AVERAGE(sL, sR);
      prevlCount = lCount;
      prevrCount = rCount;

      path[0] += s * cos(pose[2]);
      path[1] += s * sin(pose[2]);

      pose[0] = startX + path[0];
      pose[1] = startY + path[1];

      // deviation = VectorAdd(deviation, Orth(path, targetPath));  // perhaps not needed

      dist = Norm(Proj(path, targetPath));
      dev = Norm(Orth(path, targetPath));
      targetDist = sqrt((targetX - pose[0]) * (targetX - pose[0]) + (targetY - pose[1]) * (targetY - pose[1]));

      // veering left -> increase left motor power and reduce right motor power and vice versa
      // Restrict the difference between them to limit overcorrection
      if (!breaking) {
        UpdateMotorPower(targetPath, path, leftPower, rightPower, reverse, dev, motorPower);
      }

      // debug
      // Serial.print(lCount);
      // Serial.print("\t");
      // Serial.print(rCount);
      // Serial.print("\t");
      // Serial.print(targetCount);
      // Serial.print("\t");
      Serial.print(s);
      Serial.print("\t");
      Serial.print(pose[2]);
      Serial.print("\t\t");
      Serial.print(dev);
      Serial.print("\t");
      Serial.print(dist);
      Serial.print("\t");
      Serial.print(leftPower);
      Serial.print("\t");
      Serial.print(rightPower);
      Serial.print("\t");
      Serial.print(path[0]);
      Serial.print("\t");
      Serial.print(path[1]);
      Serial.print("\t");
      Serial.println(colorNames[color]);
      // Serial.print("\t");
      // printCheck();
    }
    time = millis() - startTime;
  }
  raiseMarker();
  digitalWrite(indicatorLED, LOW);

  // Serial.print("Final Position: (");
  // Serial.print(((pose[0] / countsPerRev) * wheelCirc) / 2.54);
  // Serial.print(", ");
  // Serial.print(((pose[1] / countsPerRev) * wheelCirc) / 2.54);
  // Serial.print(")in, ");
  // Serial.print(pose[2]);
  // Serial.println("rad\n");
}

//-----1-----
//-----^-----
//-4--< >--2-
//-----v-----
//-----3-----
double Close(int currSquare[], int targetDir, double sensorPose[]) {
  double score = 0.0;

  switch (targetDir) {
    case 1:
      score = ((currSquare[1] * SIDELEN) + (SIDELEN / 2.0)) - sensorPose[1];
      score = fabs(SIDELEN - score);  // * sin(pose[2]));
      break;
    case 2:
      score = ((currSquare[0] * SIDELEN) + (SIDELEN / 2.0)) - sensorPose[0];
      score = fabs(SIDELEN - score);  // * cos(pose[2]));
      break;
    case 3:
      score = sensorPose[1] - ((currSquare[1] * SIDELEN) - (SIDELEN / 2.0));
      score = fabs(SIDELEN - score);  // * sin(pose[2]));
      break;
    case 4:
      score = sensorPose[0] - ((currSquare[1] * SIDELEN) - (SIDELEN / 2.0));
      score = fabs(SIDELEN - score);  // * cos(pose[2]));
      break;
    default:
      break;
  }

  return score;
}

void UpdatePose(int x, int y) {
  Serial.print("UpdatePose( ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.println(" )");

  double encWeight = 0.7;
  double sensorWeight = 0.3;

  double square[2] = { x * SIDELEN, y * SIDELEN };
  double* sensorPose = GetSensorPose(pose);
  double diff[2] = { sensorPose[0] - pose[0], sensorPose[1] - pose[1] };
  // pose = sensorPose - diff

  // find new pose by taking weighted average of our current pose and the target one
  double newPose[2] = { sensorPose[0] * encWeight + square[0] * sensorWeight,
                        sensorPose[1] * encWeight + square[1] * sensorWeight };

  // constrain new pose to be within the target square
  // newPose[0] = constrain(newPose[0], square[0] - (SIDELEN / 2.0), square[0] + (SIDELEN / 2.0));
  // newPose[1] = constrain(newPose[1], square[1] - (SIDELEN / 2.0), square[1] + (SIDELEN / 2.0));

  pose[0] = newPose[0] - diff[0];
  pose[1] = newPose[1] - diff[1];


  delete[] sensorPose;
}

void EvaluatePose(uint8_t color, bool clear = false) {
  double* sensorPose = GetSensorPose(pose);
  int square[2] = { sensorPose[0] / SIDELEN, sensorPose[1] / SIDELEN };
  int closeLimit = 2;
  int farLimit = 3;
  double scores[4] = { 0.0 };

  static double collectiveScores[4] = { 0.0 };
  static int matches[4] = { 0 };

  if (clear) {
    for (int i = 0; i < 4; i++) {
      matches[i] = 0;
      collectiveScores[i] = 0.0;
    }
    delete[] sensorPose;
    return;
  }

  // double check color mismatch
  if ((square[0] + square[1]) % 2 == 0 && color == 2) {
    delete[] sensorPose;
    return;
  } else if ((square[0] + square[1]) % 2 != 0 && color == 1) {
    delete[] sensorPose;
    return;
  }

  for (int i = 0; i < 4; i++) {
    scores[i] = Close(square, i, sensorPose);
    if (scores[i] > (SIDELEN * 0.85)) {  // closer than 15% as threshold
      matches[i]++;
    }
    // collectiveScores[i] += scores[i];
  }
  delete[] sensorPose;

  for (int i = 0; i < 4; i++) {
    // int index = maxIndex(collectiveScores, 4);
    if (matches[i] >= 2) {
      switch (i) {
        case 1:
          UpdatePose(square[0], square[1] + 1);
          break;
        case 2:
          UpdatePose(square[0] + 1, square[1]);
          break;
        case 3:
          UpdatePose(square[0], square[1] - 1);
          break;
        case 4:
          UpdatePose(square[0] - 1, square[1]);
          break;
        default:
          break;
        
        matches[i] = 0;
      }
      break;
    } else {
      // scores[index] = 0.0;
    }
  }
}

void UpdateMotorPower(Vector<double>& targetPath, Vector<double>& path, int& leftPower, int& rightPower, bool reverse, double dev, int basePower) {
  double propGain = 0.5;
  int propMax = 40;
  int propMin = 6;

  if (Cross(targetPath, path) > 0) {
    if (!reverse) {
      leftPower = basePower + (int)(dev * propGain);
      rightPower = basePower - (int)(dev * propGain);
    } else {
      leftPower = basePower - (int)(dev * propGain);
      rightPower = basePower + (int)(dev * propGain);
    }
  } else {
    if (!reverse) {
      leftPower = basePower - (int)(dev * propGain);
      rightPower = basePower + (int)(dev * propGain);
    } else {
      leftPower = basePower + (int)(dev * propGain);
      rightPower = basePower - (int)(dev * propGain);
    }
  }

  // put a floor on differential to make sure we can correct small deviations
  if (dev > 1.0 && (ABS((leftPower - rightPower))) < (propMin * 2)) {
    if (Cross(targetPath, path) > 0) {
      if (!reverse) {
        leftPower += propMin;
        rightPower -= propMin;
      } else {
        leftPower -= propMin;
        rightPower += propMin;
      }
    } else {
      if (!reverse) {
        leftPower -= propMin;
        rightPower += propMin;
      } else {
        leftPower += propMin;
        rightPower -= propMin;
      }
    }
  }

  // Put a limit on the difference between motor power to prevent overcorrection
  if ((ABS((leftPower - rightPower))) > propMax) {
    if (leftPower > rightPower) {
      leftPower = basePower + (propMax / 2);
      rightPower = basePower - (propMax / 2);
    } else {
      leftPower = basePower - (propMax / 2);
      rightPower = basePower + (propMax / 2);
    }
  }

  if (!reverse) {
    motors.rightDrive(rightPower);
    motors.leftDrive(leftPower);
  } else {
    motors.rightDrive(-1 * rightPower);
    motors.leftDrive(-1 * leftPower);
  }
}

// Random Build - pick 2 points at random and draw a line between them
// this version for the 5x5 in grid
void RandomBuild(double sideLength, int width, int height) {
  int currX = 0;
  int currY = 0;

  if (pose[0] > 0.0) {
    currX = (int)((pose[0] + (SIDELEN / 2.0)) / SIDELEN);
  } else {
    currX = (int)((pose[0] - (SIDELEN / 2.0)) / SIDELEN);
  }
  if (pose[1] > 0.0) {
    currY = (int)((pose[1] + (SIDELEN / 2.0)) / SIDELEN);
  } else {
    currY = (int)((pose[1] - (SIDELEN / 2.0)) / SIDELEN);
  }
  // int currX = 0;
  // int currY = 0;
  int xStart = 0;
  int yStart = 0;
  int xEnd = 0;
  int yEnd = 0;
  bool dir1 = random(0, 2);
  bool dir2 = random(0, 2);

  if (dir1) {
    xStart = random(-1 * (width / 2), (width / 2) + 1);
    xStart = scale(currX, xStart, 2);
    yStart = currY;
  } else {
    xStart = currX;
    yStart = random(-1 * (height / 2), (height / 2) + 1);
    yStart = scale(currY, yStart, 2);
  }

  if (dir2) {
    do {
      xEnd = random(-1 * (width / 2), (width / 2) + 1);
      xEnd = scale(xStart, xEnd, 2);
    } while (xEnd == xStart);  // prevent random from starting and ending at the same point

    yEnd = yStart;
  } else {
    xEnd = xStart;

    do {
      yEnd = random(-1 * (height / 2), (height / 2) + 1);
      yEnd = scale(yStart, yEnd, 2);
    } while (yEnd == yStart);  // prevent random from starting and ending at the same point
  }

  Serial.print("Current Point: (");
  Serial.print(currX);
  Serial.print(", ");
  Serial.print(currY);
  Serial.println(")");

  Serial.print("Start Point: (");
  Serial.print(xStart);
  Serial.print(", ");
  Serial.print(yStart);
  Serial.println(")");

  Serial.print("End Point: (");
  Serial.print(xEnd);
  Serial.print(", ");
  Serial.print(yEnd);
  Serial.println(")\n");

  driveToPoint(xStart * sideLength, yStart * sideLength, SLOW, false);
  delay(200);
  driveToPoint(xEnd * sideLength, yEnd * sideLength, SLOW, true);
  delay(200);
}

// Version of RandomBuild where lines stay connected
void RandomBuild2(double sideLength, int width, int height) {
  int xStart = 0;
  int yStart = 0;

  if (pose[0] > 0.0) {
    xStart = (int)((pose[0] + (SIDELEN / 2.0)) / SIDELEN);
  } else {
    xStart = (int)((pose[0] - (SIDELEN / 2.0)) / SIDELEN);
  }
  if (pose[1] > 0.0) {
    yStart = (int)((pose[1] + (SIDELEN / 2.0)) / SIDELEN);
  } else {
    yStart = (int)((pose[1] - (SIDELEN / 2.0)) / SIDELEN);
  }

  int xEnd = 0;
  int yEnd = 0;
  bool dir = random(0, 2);

  if (dir) {
    do {
      xEnd = random(-1 * (width / 2), (width / 2) + 1);
      xEnd = scale(xStart, xEnd, 2);
    } while (xEnd == xStart);  // prevent random from starting and ending at the same point

    yEnd = yStart;
  } else {
    xEnd = xStart;

    do {
      yEnd = random(-1 * (height / 2), (height / 2) + 1);
      yEnd = scale(yStart, yEnd, 2);
    } while (yEnd == yStart);  // prevent random from starting and ending at the same point
  }

  Serial.print("Start Point: (");
  Serial.print(xStart);
  Serial.print(", ");
  Serial.print(yStart);
  Serial.println(")");

  Serial.print("End Point: (");
  Serial.print(xEnd);
  Serial.print(", ");
  Serial.print(yEnd);
  Serial.println(")\n");

  if (sqrt(pow(pose[0] - (xStart * SIDELEN), 2) + pow(pose[1] - (yStart * SIDELEN), 2)) > (SIDELEN * 0.15)) {
    // recenter if far from center of square
    driveToPoint(xStart * sideLength, yStart * sideLength, SLOW, false);
    delay(200);
  }
  driveToPoint(xEnd * sideLength, yEnd * sideLength, SLOW, true);
  delay(200);
}