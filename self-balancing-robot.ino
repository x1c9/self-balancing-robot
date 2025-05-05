#include <Wire.h>

#define IMU_ADDR 0x68

// Motor A connections
#define ENA 9
#define IN1 8
#define IN2 7

// Motor B connections
#define ENB 3
#define IN3 5
#define IN4 4

const float MICROSECOND = 1000000.0f;

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

float dt, timer = 0.0f;
float prev_error, integral, kp, ki, kd;
float throttle;

void gyro_signals(void);

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);

void setup() {
  kp = 1.0f;
  ki = 1.0f;
  kd = 1.0f;

  // Set all the motor control pins to outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(57600);
  Wire.setClock(400000); // Set the clock speed of I2C
  Wire.begin();
  delay(250);

  // Start the gyro in power mode
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Perform the calibration measurements
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationPitch += RatePitch;
    delay(1);
  }

  // Calculate the calibration values
  RateCalibrationPitch /= 2000;
  LoopTimer = micros();
}

void loop() {
  dt = (float)(micros() - LoopTimer) / MICROSECOND;

  gyro_signals();
  RatePitch -= RateCalibrationPitch;
  
  // Start the iteration for the Kalman filter with the pitch angle
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  Serial.print("Pitch angle [°] = ");
  Serial.println(KalmanAnglePitch);
  pid();
  driveMotors();
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}

void gyro_signals(void) {
  // Switch on the low-pass filter
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Configure the accelerometer output
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Pull the accelerometer measurements from the sensor
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(IMU_ADDR, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Set the sensitivity scale factor
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  // Access registers storing gyro measurements
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(IMU_ADDR, 6);
  int16_t GyroY = Wire.read() << 8 | Wire.read(); // Read the gyro measurements around the Y axis

  // Convert the measurement units to °/s
  RatePitch = (float) GyroY / 65.5;

  // Convert the measurements to physical values
  AccX = (float) AccXLSB / 4096 - 0.05;
  AccY = (float) AccYLSB / 4096 + 0.03;
  AccZ = (float) AccZLSB / 4096 - 0.03;

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

void pid(void) {
  float error = (float)(-KalmanAnglePitch);

  integral += error * dt;
  integral = constrain(integral, -100, 100);

  float derivative = (float)(error - prev_error) / dt;
  prev_error = error;

  throttle = (float)(kp * error + ki * integral + kd * derivative);

  throttle = constrain(throttle, -255.0f, 255.0f);
}

void driveMotors() {
  if (throttle > 0.0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  float pwm = (float)abs(throttle);
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
}
