#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include "Kalman.h"

// --- IMU and Kalman Filter ---
MPU6050 mpu;
Kalman kalmanX;   // Roll filter
Kalman kalmanY;   // Pitch filter

// --- Servos ---
Servo servoRoll;
Servo servoPitch;
Servo servoYaw;

// --- Timing ---
unsigned long timerMicros;
double kalAngleX, kalAngleY;
double gyroXangle, gyroYangle, gyroZangle;
double compAngleX, compAngleY;

// --- Gains ---
#define ROLL_GAIN   1.5
#define PITCH_GAIN  1.5
#define YAW_GAIN    2.0
#define ALPHA       0.98   // complementary filter blend factor

// --- Joystick pins ---
// Joystick 1: pitch/roll
const int joy1RollPin  = A0;
const int joy1PitchPin = A1;
const int btnOverride1 = 4;   // active LOW

// Joystick 2: roll/pitch/yaw
const int joy2RollPin  = A2;
const int joy2PitchPin = A5;
const int joy2YawPin   = A3;  // NEW: yaw axis
const int btnOverride2 = 5;   // active LOW

// --- Helpers ---
static inline double rad2deg(double r) { return r * (180.0 / PI); }

void setup() {
  Serial.begin(115200);
  Wire.begin();


  // Initialize MPU
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 Connected");

  // Calibration offsets
  mpu.setXAccelOffset(-820);
  mpu.setYAccelOffset(1633);
  mpu.setZAccelOffset(1299);
  mpu.setXGyroOffset(102);
  mpu.setYGyroOffset(11);
  mpu.setZGyroOffset(27);

  // Initial sensor read
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double accXangle = rad2deg(atan2((double)ay, (double)az));
  double accYangle = rad2deg(atan2(-(double)ax, sqrt((double)ay * ay + (double)az * az)));

  kalmanX.setAngle(accXangle);
  kalmanY.setAngle(accYangle);

  gyroXangle = accXangle;
  gyroYangle = accYangle;
  gyroZangle = 0.0;

  compAngleX = accXangle;
  compAngleY = accYangle;

  // Attach servos
  servoRoll.attach(9);
  servoPitch.attach(8);
  servoYaw.attach(7);

  // Joystick buttons
  pinMode(btnOverride1, INPUT_PULLUP);
  pinMode(btnOverride2, INPUT_PULLUP);

  timerMicros = micros();
}

void loop() {
  // --- Sensor read ---
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = micros();
  double dt = (double)(now - timerMicros) / 1e6;
  timerMicros = now;
  if (dt <= 0) dt = 1e-3;

  double gyroXrate = (double)gx / 131.0;
  double gyroYrate = (double)gy / 131.0;
  double gyroZrate = (double)gz / 131.0;

  double accXangle = rad2deg(atan2((double)ay, (double)az));
  double accYangle = rad2deg(atan2(-(double)ax, sqrt((double)ay * ay + (double)az * az)));

  // Integrate gyro
  gyroXangle += gyroXrate * dt;
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;

  // Complementary filter
  compAngleX = ALPHA * (compAngleX + gyroXrate * dt) + (1.0 - ALPHA) * accXangle;
  compAngleY = ALPHA * (compAngleY + gyroYrate * dt) + (1.0 - ALPHA) * accYangle;

  // Kalman fusion
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, dt);
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, dt);

  // --- Override logic ---
  bool override1 = (digitalRead(btnOverride1) == LOW);
  bool override2 = (digitalRead(btnOverride2) == LOW);

  int servoRollAngle;
  int servoPitchAngle;
  int servoYawAngle;

  if (override1 || override2) {
    int rollRaw  = override2 ? analogRead(joy2RollPin)  : analogRead(joy1RollPin);
    int pitchRaw = override2 ? analogRead(joy2PitchPin) : analogRead(joy1PitchPin);

    servoRollAngle  = map(rollRaw,  0, 1023, 0, 180);
    servoPitchAngle = map(pitchRaw, 0, 1023, 0, 180);

    if (override2) {
      int yawRaw = analogRead(joy2YawPin);
      servoYawAngle = map(yawRaw, 0, 1023, 0, 180);
    } else {
      servoYawAngle = map((long)(-gyroZangle * YAW_GAIN), -180, 180, 0, 180);
    }

    Serial.print("Manual Override ");
    Serial.print(override2 ? "(Joy2)" : "(Joy1)");
    Serial.print(" -> rollRaw: "); Serial.print(rollRaw);
    Serial.print(" pitchRaw: "); Serial.print(pitchRaw);
    if (override2) {
      Serial.print(" yawRaw: "); Serial.print(analogRead(joy2YawPin));
    }
  } else {
    // Automatic stabilization
    servoRollAngle  = map((long)(-kalAngleX * ROLL_GAIN),  -90, 90, 0, 180);
    servoPitchAngle = map((long)(-kalAngleY * PITCH_GAIN), -90, 90, 0, 180);
    servoYawAngle   = map((long)(-gyroZangle * YAW_GAIN), -180, 180, 0, 180);
  }

  // --- Drive servos ---
  servoRollAngle  = constrain(servoRollAngle, 0, 180);
  servoPitchAngle = constrain(servoPitchAngle, 0, 180);
  servoYawAngle   = constrain(servoYawAngle, 0, 180);

  servoRoll.write(servoRollAngle);
  servoPitch.write(servoPitchAngle);
  servoYaw.write(servoYawAngle);

  // --- Debug ---
  Serial.print("\tKF Roll: ");  Serial.print(kalAngleX, 3);
  Serial.print(" KF Pitch: ");  Serial.print(kalAngleY, 3);
  Serial.print(" GyroYaw: ");   Serial.print(gyroZangle, 3);
  Serial.print(" | ServoRoll: ");  Serial.print(servoRollAngle);
  Serial.print(" ServoPitch: ");   Serial.print(servoPitchAngle);
  Serial.print(" ServoYaw: ");     Serial.println(servoYawAngle);

  delay(10); // ~100 Hz
}
