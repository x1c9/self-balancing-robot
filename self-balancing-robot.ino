#include <Wire.h>

// Declare the global variables
float RatePitch;

// Define the calibration variables
float RateCalibrationPitch;
int RateCalibrationNumber;

// Define the accelerometer variables
float AccX, AccY, AccZ;
float AnglePitch;

uint32_t LoopTimer;

float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2;

float Kalman1DOutput[] = {0, 0};

double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;
double setpoint = 0.0;

// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;

// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

void gyro_signals(void);

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);

double pid(double error);

void setup() {
  kp = 1;
  ki = 0;
  kd = 0;

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  Serial.begin(57600);
  Wire.setClock(400000); // Set the clock speed of I2C
  Wire.begin();
  delay(250);

  // Start the gyro in power mode
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Perform the calibration measurements
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000;RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationPitch += RatePitch;
    delay(1);
  }

  // Calculate the calibration values
  RateCalibrationPitch /= 2000;
  LoopTimer = micros();
}

void loop() {
  gyro_signals();
  RatePitch -= RateCalibrationPitch;
  
  // Start the iteration for the Kalman filter with the pitch angle
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  Serial.print("Pitch angle [°] = ");
  Serial.println(KalmanAnglePitch);
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}

void gyro_signals(void) {
  // Switch on the low-pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Configure the accelerometer output
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Pull the accelerometer measurements from the sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Set the sensitivity scale factor
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  // Access registers storing gyro measurements
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);
  int16_t GyroY = Wire.read() << 8 | Wire.read(); // Read the gyro measurements around the Y axis

  // Convert the measurement units to °/s
  RatePitch = (float)GyroY/65.5;

  // Convert the measurements to physical values
  AccX = (float)AccXLSB/4096 - 0.05;
  AccY = (float)AccYLSB/4096 + 0.03;
  AccZ = (float)AccZLSB/4096 - 0.03;

  // Calculate the absolute angle
  AnglePitch = -atan(AccX/sqrt(AccY*AccY + AccZ*AccZ)) * 1/(3.142/180); 
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1/(1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

double pid(double error) {
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}