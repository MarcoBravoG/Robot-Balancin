#include <Wire.h>
#include <MPU6050.h>

// ================= CONFIGURACIÓN MOTORES L298D =================
#define ENA 25
#define IN1 26
#define IN2 27
#define ENB 33
#define IN3 32
#define IN4 14

// ================= MPU6050 =================
MPU6050 mpu;
float angleY = 0;

// ================= PID =================
float setpoint = -1.2;     // AJUSTA ENTRE -2 y +2
float Kp = 15.0;
float Ki = 0.3;
float Kd = 0.9;

float error = 0, lastError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// ================= CONSTANTES =================
const float ACC_SCALE = 16384.0;
const float GYRO_SCALE = 131.0;
const float ANGULO_MAX = 40.0;
const int VEL_MAX = 180;
const int PWM_MIN = 40;

// ================= FILTRO COMPLEMENTARIO =================
float compAngleY = 0;
float alpha = 0.98;

// ================= TIEMPOS =================
unsigned long lastMPUTime = 0;
unsigned long lastPIDTime = 0;
unsigned long lastPrint = 0;

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  detenerMotores();

  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("ERROR MPU6050");
    while (1);
  }

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  Serial.println("ROBOT BALANCIN INICIADO");
  Serial.println("Setpoint,Angle,Error,Output");
  delay(2000);
}

// ================= LOOP =================
void loop() {
  leerMPU();

  if (abs(angleY) > ANGULO_MAX) {
    detenerMotores();
    integral = 0;
    delay(1000);
    return;
  }

  calcularPID();
  moverMotores(output);

  if (millis() - lastPrint > 20) {
    Serial.print(setpoint); Serial.print(",");
    Serial.print(angleY, 2); Serial.print(",");
    Serial.print(error, 2); Serial.print(",");
    Serial.println(output, 1);
    lastPrint = millis();
  }

  delay(5);
}

// ================= MPU =================
void leerMPU() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accX = ax / ACC_SCALE;
  float accY = ay / ACC_SCALE;
  float accZ = az / ACC_SCALE;

  float accAngle = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
  float gyroY = gy / GYRO_SCALE;

  unsigned long now = millis();
  float dt = (now - lastMPUTime) / 1000.0;
  if (dt <= 0 || dt > 0.05) dt = 0.01;

  if (lastMPUTime == 0)
    compAngleY = accAngle;
  else
    compAngleY = alpha * (compAngleY + gyroY * dt) + (1 - alpha) * accAngle;

  angleY = compAngleY;
  lastMPUTime = now;
}

// ================= PID =================
void calcularPID() {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  if (dt <= 0 || dt > 0.05) dt = 0.01;
  lastPIDTime = now;

  error = setpoint - angleY;

  integral += error * dt;
  integral = constrain(integral, -10, 10);

  derivative = (error - lastError) / dt;

  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  output = constrain(output, -VEL_MAX, VEL_MAX);

  lastError = error;
}

// ================= MOTORES =================
void moverMotores(int vel) {
  vel = constrain(vel, -VEL_MAX, VEL_MAX);

  if (vel > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (vel < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else {
    detenerMotores();
    return;
  }

  int pwm = abs(vel);
  if (pwm < PWM_MIN) pwm = PWM_MIN;

  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
}

void detenerMotores() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}