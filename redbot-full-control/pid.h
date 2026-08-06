// PID
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