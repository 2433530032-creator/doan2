
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>

// ================= WIFI RIENG =================
const char* AP_SSID = "BalanceRobot";
const char* AP_PASS = "Balance123";

WebServer server(80);

// ================= MPU6050 =================
#define MPU_ADDR 0x68
#define SDA_PIN 21
#define SCL_PIN 22

// ================= TB6612FNG =================
#define PWMA 32
#define AIN1 25
#define AIN2 33
#define PWMB 13
#define BIN1 27
#define BIN2 14
#define STBY 26

// ================= ENCODER =================
#define ENC_L_A 16
#define ENC_L_B 17
#define ENC_R_A 18
#define ENC_R_B 19

const int LEFT_MOTOR_SIGN  = 1;
const int RIGHT_MOTOR_SIGN = 1;
const int LEFT_ENCODER_SIGN = 1;
const int RIGHT_ENCODER_SIGN = 1;
const int BALANCE_SIGN = 1;

const int PWM_MAX = 255;
const int MOTOR_MIN_PWM = 25;
const uint32_t CONTROL_US = 5000;

// ================= PID GOC =================
float Kp = 18.0f;
float Ki = 0.0f;
float Kd = 0.80f;

float integral = 0.0f;
float gyroBias = 0.0f;
float angleOffset = 0.0f;
float angle = 0.0f;
float gyroRate = 0.0f;
float outputPWM = 0.0f;
float targetAngle = 0.0f;

const float INTEGRAL_LIMIT = 50.0f;

volatile long encL = 0;
volatile long encR = 0;

long prevL = 0;
long prevR = 0;

float speedL = 0.0f;
float speedR = 0.0f;
float positionTicks = 0.0f;

uint32_t lastControl = 0;
bool sensorOK = false;
bool balancing = false;

// ================= KALMAN =================
class KalmanAngle {
public:
  float Qangle = 0.001f;
  float Qbias = 0.003f;
  float Rmeasure = 0.03f;

  float a = 0.0f;
  float b = 0.0f;
  float P00 = 0.0f, P01 = 0.0f;
  float P10 = 0.0f, P11 = 0.0f;

  void reset(float value) {
    a = value;
    b = 0;
    P00 = P01 = P10 = P11 = 0;
  }

  float update(float measured, float rate, float dt) {
    float r = rate - b;
    a += dt * r;

    P00 += dt * (dt * P11 - P01 - P10 + Qangle);
    P01 -= dt * P11;
    P10 -= dt * P11;
    P11 += Qbias * dt;

    float S = P00 + Rmeasure;
    if (S <= 0) return a;

    float K0 = P00 / S;
    float K1 = P10 / S;
    float y = measured - a;

    a += K0 * y;
    b += K1 * y;

    float p00 = P00;
    float p01 = P01;

    P00 -= K0 * p00;
    P01 -= K0 * p01;
    P10 -= K1 * p00;
    P11 -= K1 * p01;

    return a;
  }
};

KalmanAngle kalman;

// ================= MPU6050 =================
bool readMPU(
  int16_t &ax, int16_t &ay, int16_t &az,
  int16_t &gx, int16_t &gy, int16_t &gz
) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14,
                       (uint8_t)true) != 14) return false;

  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());

  Wire.read();
  Wire.read();

  gx = (int16_t)((Wire.read() << 8) | Wire.read());
  gy = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = (int16_t)((Wire.read() << 8) | Wire.read());

  return true;
}

// Cong thuc nay giu cach tinh goc da su dung trong code cua ban.
// Can xac nhan huong truc cam bien thuc te tren xe.
float accelAngle(int16_t ax, int16_t ay, int16_t az) {
  return atan2f((float)ay, (float)az) * 180.0f / PI;
}

bool initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  if (Wire.endTransmission() != 0) return false;

  delay(50);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0);
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0);
  return Wire.endTransmission() == 0;
}

bool calibrateMPU() {
  long sumG = 0;
  float sumA = 0;
  int valid = 0;

  Serial.println("Hieu chuan: giu xe dung yen va gan thang dung.");

  for (int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;

    if (readMPU(ax, ay, az, gx, gy, gz)) {
      sumG += gx;
      sumA += accelAngle(ax, ay, az);
      valid++;
    }
    delay(2);
  }

  if (valid < 250) return false;

  gyroBias = (float)sumG / valid;
  angleOffset = sumA / valid;

  angle = 0;
  kalman.reset(0);
  return true;
}

// ================= ENCODER =================
void IRAM_ATTR isrL() {
  encL += digitalRead(ENC_L_B) ? 1 : -1;
}

void IRAM_ATTR isrR() {
  encR += digitalRead(ENC_R_B) ? 1 : -1;
}

// ================= MOTOR =================
void motorOne(int pwmPin, int in1, int in2,
              int cmd, int sign) {
  int value = constrain(cmd * sign, -PWM_MAX, PWM_MAX);
  int pwm = abs(value);

  if (pwm > 0 && pwm < MOTOR_MIN_PWM) pwm = MOTOR_MIN_PWM;

  if (value > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (value < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwm = 0;
  }

  analogWrite(pwmPin, pwm);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  outputPWM = 0;
}

void setMotors(int left, int right) {
  digitalWrite(STBY, HIGH);
  motorOne(PWMA, AIN1, AIN2, left, LEFT_MOTOR_SIGN);
  motorOne(PWMB, BIN1, BIN2, right, RIGHT_MOTOR_SIGN);
}

// ================= PID =================
float updatePID(float dt) {
  float error = targetAngle - angle;

  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float out = Kp * error + Ki * integral - Kd * gyroRate;
  return constrain(out * BALANCE_SIGN, -PWM_MAX, PWM_MAX);
}

// ================= WEB API =================
void handleRoot() {
  server.send(200, "text/plain",
              "BalanceRobot OK. Endpoints: /data /start /stop /pid");
}

void handleData() {
  String json = "{";
  json += "\"angle\":" + String(angle, 2) + ",";
  json += "\"target\":" + String(targetAngle, 2) + ",";
  json += "\"left\":" + String(speedL, 1) + ",";
  json += "\"right\":" + String(speedR, 1) + ",";
  json += "\"position\":" + String(positionTicks, 0) + ",";
  json += "\"output\":" + String(outputPWM, 1) + ",";
  json += "\"enabled\":";
  json += balancing ? "true" : "false";
  json += "}";

  server.send(200, "application/json", json);
}

void handlePID() {
  if (!server.hasArg("kp") || !server.hasArg("ki") ||
      !server.hasArg("kd")) {
    server.send(400, "text/plain", "Missing kp, ki, kd");
    return;
  }

  float p = server.arg("kp").toFloat();
  float i = server.arg("ki").toFloat();
  float d = server.arg("kd").toFloat();

  if (!isfinite(p) || !isfinite(i) || !isfinite(d) ||
      p < 0 || p > 100 || i < 0 || i > 100 ||
      d < 0 || d > 20) {
    server.send(400, "text/plain", "PID out of range");
    return;
  }

  Kp = p;
  Ki = i;
  Kd = d;
  integral = 0;

  server.send(200, "text/plain", "PID updated");
}

void handleStart() {
  if (!sensorOK) {
    server.send(400, "text/plain", "MPU6050 not ready");
    return;
  }

  if (fabsf(angle) > 8.0f) {
    server.send(400, "text/plain", "Hold robot upright first");
    return;
  }

  noInterrupts();
  prevL = encL;
  prevR = encR;
  interrupts();

  positionTicks = 0;
  integral = 0;
  lastControl = micros();
  balancing = true;

  server.send(200, "text/plain", "Balance enabled");
}

void handleStop() {
  balancing = false;
  stopMotors();
  server.send(200, "text/plain", "Motors stopped");
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(STBY, HIGH);
  stopMotors();

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, RISING);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(50);

  sensorOK = initMPU() && calibrateMPU();

  // ESP32 tu phat Wi-Fi, khong can router.
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  bool apOK = WiFi.softAP(AP_SSID, AP_PASS, 6, false, 4);

  if (apOK) {
    Serial.print("Wi-Fi: ");
    Serial.println(AP_SSID);
    Serial.print("Gateway: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Loi tao Wi-Fi AP");
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/pid", HTTP_GET, handlePID);
  server.on("/start", HTTP_GET, handleStart);
  server.on("/stop", HTTP_GET, handleStop);
  server.begin();

  lastControl = micros();

  Serial.println(sensorOK ? "MPU6050 OK" : "MPU6050 ERROR");
  Serial.println("WebServer ready");
}

// ================= LOOP =================
void loop() {
  server.handleClient();

  uint32_t now = micros();
  if ((uint32_t)(now - lastControl) < CONTROL_US) return;

  float dt = (uint32_t)(now - lastControl) / 1000000.0f;
  lastControl = now;

  if (dt <= 0 || dt > 0.05f) {
    stopMotors();
    return;
  }

  int16_t ax, ay, az, gx, gy, gz;

  if (!readMPU(ax, ay, az, gx, gy, gz)) {
    sensorOK = false;
    balancing = false;
    stopMotors();
    return;
  }

  sensorOK = true;
  gyroRate = ((float)gx - gyroBias) / 131.0f;

  float measured = accelAngle(ax, ay, az) - angleOffset;
  angle = kalman.update(measured, gyroRate, dt);

  long l, r;
  noInterrupts();
  l = encL;
  r = encR;
  interrupts();

  long dl = l - prevL;
  long dr = r - prevR;
  prevL = l;
  prevR = r;

  speedL = dl * LEFT_ENCODER_SIGN / dt;
  speedR = dr * RIGHT_ENCODER_SIGN / dt;
  positionTicks += (
    dl * LEFT_ENCODER_SIGN + dr * RIGHT_ENCODER_SIGN
  ) * 0.5f;

  if (fabsf(angle) > 50.0f) {
    balancing = false;
    integral = 0;
    stopMotors();
    return;
  }

  if (!balancing) {
    stopMotors();
    return;
  }

  outputPWM = updatePID(dt);
  int cmd = (int)roundf(outputPWM);
  setMotors(cmd, cmd);
}
