#include <Wire.h>

// Motor A connections
#define ENA 9
#define IN1 8
#define IN2 7

// Motor B connections
#define ENB 3
#define IN3 5
#define IN4 4

// Khai báo biến toàn cục
float dt = 0.004f;            // Mặc định 4ms
float error = 0.0f;
float integral = 0.0f;
float derivative = 0.0f;
float kp = 500.0f;
float ki = 0.0f;
float kd = 0.0f;
float throttle = 0.0f;
float setpoint = 0.0f;        // Góc mong muốn (thường là 0 độ)
float lastInput = 0.0f;       // Lưu giá trị input (KalmanAnglePitch) trước đó
float lastTime = 0.0f;        // Lưu thời gian tính toán PID trước đó
float sampleTime = 0.004f;    // Thời gian lấy mẫu 4ms (250Hz)
const float MICROSECOND = 1000000.0f;
const float MIN_DT = 0.001f;  // Thời gian tối thiểu 1ms
const float MAX_ANGLE = 35.0f; // Ngưỡng góc nghiêng tối đa
const float MIN_PWM = 5.0f;   // Ngưỡng PWM tối thiểu
float max_integral;
float pwm;

float RatePitch;
float RateCalibrationPitch;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AnglePitch;
uint32_t LoopTimer;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
float Kalman1DOutput[] = {0, 0};

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState += dt * KalmanInput;
  KalmanUncertainty += dt * dt * 16;
  float KalmanGain = KalmanUncertainty * 1 / (KalmanUncertainty + 9);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  RatePitch=(float)GyroY/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29578;
}

void pid(void) {
  float now = (float)micros() / MICROSECOND; // Thời gian hiện tại (giây)
  float timeChange = now - lastTime;

  if (timeChange >= sampleTime) {
    float input = KalmanAnglePitch; // Góc nghiêng hiện tại
    error = setpoint - input;       // Sai số

    // Kiểm tra ngưỡng góc nghiêng
    if (abs(error) > MAX_ANGLE) {
      throttle = 0.0f;
      integral = 0.0f;
      derivative = 0.0f;
    } else {
      // Tính thành phần tích phân
      integral += ki * error * dt;
      integral = constrain(integral, -max_integral, max_integral);

      // Tính thành phần vi phân dựa trên sự thay đổi của input
      float dInput = input - lastInput;
      derivative = -kd * dInput / dt; // Dùng dInput thay vì sai số
      derivative = 0.9f * derivative + 0.1f * derivative; // Lọc thông thấp

      // Tính đầu ra PID
      float pTerm = kp * error;
      throttle = pTerm + integral + derivative;
      throttle = constrain(throttle, -255.0f, 255.0f);

      // Cập nhật lastInput và lastTime
      lastInput = input;
      lastTime = now;
    }
  }
}

void driveMotors() {
  float pwm = abs(throttle);
  if (pwm < MIN_PWM) {
    pwm = 0.0f;
  }

  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);

  if (throttle < 0.0f) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}

void setup() {
  // Khởi tạo các biến PID
  integral = 0.0f;
  derivative = 0.0f;
  lastInput = 0.0f;
  lastTime = (float)micros() / MICROSECOND;
  max_integral = 100.0f;

  // Thiết lập các chân động cơ
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();
  delay(500);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 500; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationPitch += RatePitch;
    delay(1);
  }
  RateCalibrationPitch /= 500;
  LoopTimer = micros();
}

void loop() {
  dt = (float)(micros() - LoopTimer) / MICROSECOND;
  if (dt < MIN_DT) dt = 0.004f; // Tránh dt quá nhỏ

  gyro_signals();
  RatePitch -= RateCalibrationPitch;
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  pid();
  driveMotors();

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}