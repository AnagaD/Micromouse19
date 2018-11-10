#include <Wire.h>
#include <VL6180X.h>

VL6180X sensor;

int leftWheelB = 21;
int leftWheelA = 22;
int rightWheelA = 23;
int rightWheelB = 20;
int onboardLED = 13;
int threshold; 

void setup() {
  // Pin configuration
  pinMode(leftWheelB, OUTPUT);
  pinMode(leftWheelA, OUTPUT);
  pinMode(rightWheelB, OUTPUT);
  pinMode(rightWheelA, OUTPUT);
  // Show when robot is on
  digitalWrite(onboardLED, HIGH);
  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);

  // stop continuous mode if already active
  sensor.stopContinuous();
  // in case stopContinuous() triggered a single-shot
  // measurement, wait for it to complete
  delay(300);
  // start interleaved continuous mode with period of 100 ms
  sensor.startInterleavedContinuous(100);
  threshold=sensor.readAmbientContinuous();
  
}

void setDriveSpeed(int left, int right) {
  if(left > 0) {
    analogWrite(leftWheelA, left);
    analogWrite(leftWheelB, 0);
  }
  else if (left < 0) {
    analogWrite(leftWheelB, -1*left);
    analogWrite(leftWheelA, 0);
  }
  else {
    analogWrite(leftWheelA, 0);
    analogWrite(leftWheelB, 0);
  }
  if(right > 0) {
    analogWrite(rightWheelB, right);
    analogWrite(rightWheelA, 0);
  }
  else if (right < 0) {
    analogWrite(rightWheelA, -1*right);
    analogWrite(rightWheelB, 0);
  }
  else {
    analogWrite(rightWheelA, 0);
    analogWrite(rightWheelB, 0);
  }
}

void turnRight() {
  setDriveSpeed(255,0);
}

void goStraight() {
  setDriveSpeed(100,-100);
}
void turnLeft() {
  setDriveSpeed(0,-255);
}

void loop() {
  // Acquire sensor readings
  Serial.println("Ambient: ");
  Serial.println(sensor.readAmbientContinuous());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
if(sensor.readAmbientContinuous()>threshold)
{
goStraight();
}
}

  /* TODO: Your code here!
   * Right now the robot doesn't take any of the sensor readings into account,
   * it just constantly turns right. Your job is to incorporate your sensor
   * readings into a decision about how to drive the motors!
   */
  


  // TODO: ADVANCED
  // If you want to improve your line follower further, you can use PID control.
  // Ask a HURC member about this topic once you get your line follower working!
  /* PID control loop
  // definition of error changes depending on sensors used
  error = (on.light - off.light) - THRESHOLD; // polarity might be change depending on system orientation
  // update error integration
  cumulative_error += error;
  // calculate error derivative
  error_derivative = error - prev_error;
  control = kp * error + ki * cumulative_error + kd * error_derivative;
  drive = control; // need some conversion of control to drive signal (need real values to determine this)
  setDriveSpeed(20, drive); // only need to control right wheel's speed if you keep the other constant
  // update error buffer
  prev_error = error;
  */
