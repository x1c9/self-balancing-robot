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
float prev_error = 0.0f;
float integral = 0.0f;
float derivative = 0.0f;
float kp = 100.0f;
float ki = 0.0f;
float kd = 0.0f;
float throttle = 0.0f;
float error_threshold = 30.0f;  // Ngưỡng góc nghiêng tối đa
float max_integral = 100.0f;    // Giới hạn tích phân
float derivative_filter = 0.1f; // Hệ số lọc đạo hàm
float prev_derivative = 0.0f;   // Lưu giá trị đạo hàm trước
float pwm = 0.0f;

const float MICROSECOND = 1000000.0f;
const float MIN_DT = 0.001f;    // Thời gian tối thiểu 1ms


float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
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
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}
void pid(void) {
  // Đảm bảo dt không quá nhỏ để tránh chia cho số gần 0
  if (dt < MIN_DT) dt = MIN_DT;

  float error = -KalmanAnglePitch;

  if (abs(error) > error_threshold) { // Nếu góc nghiêng vượt quá 30 độ
    throttle = 0.0; // Dừng động cơ
    integral = 0.0; // Đặt lại tích phân để tránh windup
  } else {
    integral += error * dt;
    integral = constrain(integral, -max_integral, max_integral);

    // Tính đạo hàm với bộ lọc thấp
    float raw_derivative = (error - prev_error) / dt;
  
    // Kiểm tra đạo hàm hợp lệ
    if (isnan(raw_derivative) || isinf(raw_derivative)) {
      raw_derivative = 0.0;
    }
  
    // Áp dụng bộ lọc thấp cho đạo hàm
    derivative = (1.0 - derivative_filter) * prev_derivative + derivative_filter * raw_derivative;
    prev_derivative = derivative;

    prev_error = error;

    throttle = kp * error + ki * integral + kd * derivative;
    throttle = constrain(throttle, -255, 255);
  }
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

  pwm = abs(throttle);
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
}


//////// Setup


void setup() {
  // Set all the motor control pins to outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();
}
void loop() {
  dt = (float)(micros() - LoopTimer) / MICROSECOND;
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  Serial.print("Roll Angle [°] ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch Angle [°] ");
  Serial.print(KalmanAnglePitch);
  pid();
  driveMotors();
  Serial.print(" throttle ");
  Serial.print(throttle);
  Serial.print(" pwm ");
  Serial.println(pwm);
  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}