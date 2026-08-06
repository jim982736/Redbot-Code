#include <math.h>

#define SIDELEN (((5.0 * 2.54) / (PI * 6.7)) * 192.0)
#define ABS(n) ((n < 0) ? (n * -1) : (n))
#define AVERAGE(a, b) ((a + b) / 2)
#define RAD(theta) (theta * (PI / 180.0))
#define DEG(phi) (phi * (180.0 / PI))

float wheelDiam = 6.7;             // 2.6 diam = 65mm, measured wheel to be closer to 67mm
float wheelCirc = PI * wheelDiam;  // Redbot wheel circumference = pi*D
int countsPerRev = 192;            // 48:1 gearing, 4 ticks per revolution

void flashLED(int pin, int times, int time) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(time);
    digitalWrite(pin, LOW);
    delay(time);
  }
}

// --Position estimation functions--

// Position of color sensor relative to center
double* GetSensorPose(double pose[]) {
  double* pos = new double[2];

  pos[0] = pose[0] + ((2.2 / wheelCirc) * countsPerRev) * cos(pose[2]);
  pos[1] = pose[1] + ((2.2 / wheelCirc) * countsPerRev) * sin(pose[2]);

  return pos;
}

bool CheckPose(double pose[], int color) {
  if (color == 0) {
    return true;
  }

  double* sensorPose = GetSensorPose(pose);
  int square[2] = { 0, 0 };

  if (sensorPose[0] > 0.0) {
    square[0] = (int)((sensorPose[0] + (SIDELEN / 2.0)) / SIDELEN);
  } else {
    square[0] = (int)((sensorPose[0] - (SIDELEN / 2.0)) / SIDELEN);
  }
  if (sensorPose[1] > 0.0) {
    square[1] = (int)((sensorPose[1] + (SIDELEN / 2.0)) / SIDELEN);
  } else {
    square[1] = (int)((sensorPose[1] - (SIDELEN / 2.0)) / SIDELEN);
  }

  delete[] sensorPose;

  if ((square[0] + square[1]) % 2 == 0) {
    if (color == 2) {
      return true;
    } else {
      return false;
    }
  } else {
    if (color == 1) {
      return true;
    } else {
      return false;
    }
  }
}

// Limit maximum build length to maxDist
int scale(int start, int end, int maxDist) {
  if ((abs(end - start)) > maxDist) {
    if (end > start) {
      return start + maxDist;
    } else {
      return start - maxDist;
    }
  }
  return end;
}

// Signature helper functions
typedef struct {
  double levels[6];
  double stdDevs[6];
  double avgTotal;
} Signature;

double Score(Signature& sig, double tot, double* sensorLevels) {
  double score = 0.0;
  double diff = 0.0;
  double totalWeight = 1.0;
  double stdDevWeight = 0.1;

  // std devs and total ultimately caused more innacuracy so we're only using the difference
  // between the reading and the signature
  for (int i = 0; i < 6; i++) {
    diff = fabs(sig.levels[i] - sensorLevels[i]);
    score += (1.0 - diff);  // / (sig.stdDevs[i] / stdDevWeight);
  }

  //score += (280.0 / (fabs(sig.avgTotal - tot)) * totalWeight);

  return score;
}

void PopulateSignature(Signature& sig, double* levels, double* stdDevs, double avgTotal) {
  sig.avgTotal = avgTotal;
  double stdDevTotal = 0.0;

  for (int i = 0; i < 6; i++) {
    sig.levels[i] = levels[i];
    stdDevTotal += stdDevs[i];
  }

  for (int i = 0; i < 6; i++) {
    sig.stdDevs[i] = stdDevs[i] / stdDevTotal;
  }
}

int sum(int cnt, float* vals) {
  double tot = 0.0;

  for (int i = 0; i < cnt; i++) {
    tot += vals[i];
  }

  return tot;
}

// Vector operations
double Norm(Vector<double> v) {
  return sqrt(v[0] * v[0] + v[1] * v[1]);
}

double Dot(Vector<double> a, Vector<double> b) {
  return a[0] * b[0] + a[1] * b[1];
}

// Returns magnitude of 2D cross product
double Cross(Vector<double> a, Vector<double> b) {
  return a[0] * b[1] - a[1] * b[0];
}

double projectionStorage[2];
Vector<double> Proj(Vector<double> a, Vector<double> b) {
  double factor = (Dot(a, b) / (Norm(b) * Norm(b)));
  Vector<double> result;
  result.setStorage(projectionStorage, 2);

  result[0] = factor * b[0];
  result[1] = factor * b[1];

  return result;
}

double orthogonalStorage[2];
Vector<double> Orth(Vector<double> a, Vector<double> b) {
  Vector<double> result;
  result.setStorage(orthogonalStorage, 2);
  result = Proj(a, b);

  result[0] = a[0] - result[0];
  result[1] = a[1] - result[1];

  return result;
}


double vecAddStorage[2];
Vector<double> VectorAdd(Vector<double> a, Vector<double> b) {
  Vector<double> result;
  result.setStorage(vecAddStorage, 2);

  result[0] = a[0] + b[0];
  result[1] = a[1] + b[1];

  return result;
}

// Other Helper Functions
int maxIndex(double arr[], int len) {
  int max = 0;

  for (int i = 0; i < len; i++) {
    if (arr[i] > arr[max]) {
      max = i;
    }
  }

  return max;
}