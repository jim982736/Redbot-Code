// libraries for gyro
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>

#include <RedBot.h>
#include <Servo.h>
#define AVERAGE(a, b) ((a + b) / 2)
#define ABS(n) ((n < 0) ? n * -1 : n)

// for use with driveStraight, generally use SLOW for shorter
// distances (< 20cm) and FAST for longer distances
#define SLOW 40
#define FAST 100

RedBotMotors motors;

// assign unique ID to gryo, seems like it needs to be named "gyro"
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

RedBotEncoder encoder = RedBotEncoder(A2, 10);
int buttonPin = 12;
int countsPerRev = 192;  // 48:1 gearing, 4 ticks per revolution

float wheelDiam = 6.7;             // 2.6 diam = 65mm, measured wheel to be closer to 67mm
float wheelCirc = PI * wheelDiam;  // Redbot wheel circumference = pi*D
float wheelDist = 5.0;             // approx. dist from center of redbot to center of wheels

const int servoPin = 9;
Servo marker;

struct PID {
  double integState;
  double integMax, integMin;
  double derState;
  int propGain, derGain, integGain;
};

// update values in PID based on the error
int updatePID(PID* pid, double error, double dist) {
  int prop, integ, der;

  // calculate proportional term
  prop = pid->propGain * error;

  // calculate integral state with limiting
  pid->integState += error;
  if (pid->integState > pid->integMax)
    pid->integState = pid->integMax;
  else if (pid->integState < pid->integMin)
    pid->integState = pid->integMin;

  integ = pid->integGain * pid->integState;

  // calculate derivative term
  der = pid->derGain * (pid->derState - dist);
  pid->derState = dist;

  // print values to Serial port for tuning
  /*Serial.print(pid->integState);
  Serial.print('\t');
  Serial.print(integ);
  Serial.print('\t');
  Serial.print(prop);
  Serial.print('\t');
  Serial.print(der);
  Serial.print('\t');*/

  return prop + integ + der;
}

void driveStraight(float distance, int motorPower, bool draw = true, bool reverse = false);

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  marker.write(0);
  marker.attach(servoPin);
  Serial.begin(9600);

  gyro.enableAutoRange(true);

  if (!gyro.begin()) {
    Serial.println("--gyro not detected, check wiring--");
    while (true)
      ;
  }
}

void loop(void) {
  // set the power for left & right motors on button press
  if (digitalRead(buttonPin) == LOW) {
    /*delay(200);
    marker.write(15);
    delay(2000);
    marker.write(28);
    delay(200);*/

    // Demo
    /*turnToAngleGyro(60.0);
    turnToAngleGyro(-150.0);
    driveStraight(40.0, 100);
    turnToAngleGyro(120.0);
    driveStraight(20.0, 100);
    turnToAngleGyro(120.0);
    driveStraight(40.0, 100);
    turnToAngleGyro(-120.0);
    driveStraight(20.0, 100);
    turnToAngleGyro(-30.0);*/

    // Zigzag path demo
    turnToAngleGyro(30.0);
    driveStraight(15.0, FAST);
    turnToAngleGyro(-120.0);
    driveStraight(3.25, SLOW, false);
    driveStraight(21.5, FAST);
    driveStraight(9.75, SLOW, false, true);
    turnToAngleGyro(90.0);
    /*turnToAngleGyro(150.0);
    driveStraight(25.625, FAST);
    turnToAngleGyro(-130.0);
    driveStraight(39.375, FAST);
    turnToAngleGyro(70.0);*/

    /*driveStraight(45.0, 100);
    turnToAngleGyro(-90.0);*/

    /*driveStraight(10.0, SLOW);
    marker.write(15);*/
  }
}

// counter clockwise for positive angles and clockwise for negative ones
void turnToAngleGyro(double angle) {
  marker.write(15);

  double target = angle * (PI / 180.0);
  double dist = 0.0;

  Serial.print("Target: ");
  Serial.println(angle);

  // variables for tracking distance travelled
  int prevTime = millis();
  int currTime = 0;
  double timeDiff = 0;

  // control variables
  double error = target;
  int drive = 0;
  int ceiling = 100;  // highest motor power with ability to track movement
  int finishCount = 0;

  // initialize PID controller for turning, adjust gains to tune controller
  PID pid;
  pid.propGain = 625;
  pid.integGain = 31;
  pid.derGain = 1250;
  pid.derState = 0.0;
  pid.integMax = 20.0;
  pid.integMin = -20.0;
  pid.integState = 0.0;

  delay(200);
  motors.pivot(ceiling);

  while (finishCount < 10) {
    sensors_event_t event;
    gyro.getEvent(&event);

    currTime = millis();
    timeDiff = currTime - prevTime;
    prevTime = currTime;
    timeDiff /= 1000;
    dist += timeDiff * event.gyro.z;

    //PI controller to determine motor power
    error = target - dist;
    drive = updatePID(&pid, error, dist);

    // give drive a ceiling and floor to ensure enough torque
    if (drive > ceiling) {
      drive = ceiling;
    } else if (drive < (-1 * ceiling)) {
      drive = -1 * ceiling;
    }

    if (ABS(error) < 0.01) {
      finishCount++;
    }

    // pivot with new motor power and wait 25ms for motors to respond
    motors.pivot(drive);

    /*Serial.print(error);
    Serial.print('\t');
    Serial.println(drive);*/

    delay(25);
  }
  motors.brake();
}

// drive straight a certain distance in cm
void driveStraight(float distance, int motorPower, bool draw = true, bool reverse = false) {
  delay(100);

  if (draw) {
    marker.write(28);
  } else {
    marker.write(15);
  }

  long lCount = 0;
  long rCount = 0;
  long targetCount;
  float numRev;

  double overshoot = 2.54;

  if (motorPower == FAST) {  // approximate overshoot when driving a specific distance at these values of motorPower
    overshoot = 1.54;
  } else if (motorPower == SLOW) {
    overshoot = 0.54;
  }

  // variables for tracking the left and right encoder counts
  long prevlCount = 0, prevrCount = 0;

  long lDiff, rDiff;  // diff between current encoder count and previous count

  // variables for setting left and right motor power
  int leftPower = motorPower;
  int rightPower = motorPower;

  // variable used to offset motor power on right vs left to keep straight.
  int offset = 5;  // offset amount to compensate Right vs. Left drive
  double error = 0;

  numRev = (distance - overshoot) / wheelCirc;  // calculate the target # of rotations, subtracting overshoot from distance
  targetCount = numRev * countsPerRev;          // calculate the target count

  // debug
  Serial.print("driveStraight() ");
  Serial.print(distance);
  Serial.print(" inches at ");
  Serial.print(motorPower);
  Serial.println(" power.");

  Serial.print("Target: ");
  Serial.print(numRev, 3);
  Serial.println(" revolutions.");
  Serial.println();

  // print out header
  Serial.print("Left\t");   // "Left" and tab
  Serial.print("Right\t");  // "Right" and tab
  Serial.print("Target count\t\t");
  Serial.println("Error");
  Serial.println("========================================");

  int startTime = millis();
  int time = 0;

  encoder.clearEnc(BOTH);  // clear the encoder count
  delay(200);              // short delay before starting the motors.

  // start motors
  if (reverse) {
    motors.drive(-1 * motorPower);  
  } else {
    motors.drive(motorPower);
  }

  while (AVERAGE(rCount, lCount) < targetCount) {
    lCount = encoder.getTicks(LEFT);
    rCount = encoder.getTicks(RIGHT);

    if (time >= 50) {
      time = 0;
      startTime = millis();

      // while the encoders read less than the target count -- debug print
      // the encoder values and wait -- this is a holding loop.

      /*Serial.print(lCount);
      Serial.print("\t");
      Serial.print(rCount);
      Serial.print("\t");
      Serial.print(targetCount);
      Serial.print("\t");*/

      if (reverse) {
        motors.leftDrive(-1 * leftPower);
        motors.rightDrive(-1 * rightPower);
      } else {
        motors.leftDrive(leftPower);
        motors.rightDrive(rightPower);
      }

      // calculate the rotation "speed" as a difference in the count from previous cycle.
      lDiff = (lCount - prevlCount);
      rDiff = (rCount - prevrCount);
      error = (double)ABS((lDiff - rDiff)) / 5.0;

      //Serial.println(error);

      // store the current count as the "previous" count for the next cycle.
      prevlCount = lCount;
      prevrCount = rCount;

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
    }
    time = millis() - startTime;
  }
  // now apply "brakes" to stop the motors.
  motors.brake();
}

// Unused turning function based purely on encoders
void turnToAngle(double angle, int motorPower) {
  double overshoot = 20;                                                    // measured overshoot
  double arcLength = ((angle - overshoot) / 360.0) * (wheelDist * 2 * PI);  // distance traveled by each wheel
  double numRev = arcLength / wheelCirc;

  long targetCount = numRev * countsPerRev;
  long lCount = 0;
  long rCount = 0;
  int power = ((angle > 0) ? motorPower : (motorPower * -1));

  encoder.clearEnc(BOTH);
  marker.write(15);
  delay(200);
  motors.pivot(power);

  Serial.print("Target: ");
  Serial.println(targetCount);

  while (AVERAGE(lCount, rCount) < targetCount) {
    lCount = ABS(encoder.getTicks(LEFT));
    rCount = ABS(encoder.getTicks(RIGHT));

    Serial.print(lCount);
    Serial.print("\t");
    Serial.println(rCount);
  }
  motors.brake();
  delay(200);
}