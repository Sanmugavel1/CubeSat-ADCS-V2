#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include "LittleFS.h"
#include <MadgwickAHRS.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <LoRa.h>

// ===== CONTROL TARGET =====
float targetAngle = 0.0;   // Desired roll angle
float error = 0.0;

// ================= WIFI =================
const char* ssid = "vivo Y100A";
const char* password = "sarves2006";

// ================= MPU =================
#define MPU_ADDR 0x68
bool stabilizeMode = true;

AsyncWebServer server(80);
AsyncEventSource events("/events");
JSONVar readings;

// ================= TIMING =================
unsigned long lastTime = 0;
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long lastIMUTime = 0;

unsigned long gyroDelay = 10;
unsigned long accelerometerDelay = 200;
unsigned long temperatureDelay = 1000;

// ================= FILTER =================
Madgwick filter;

// orientation
float roll, pitch, yaw;
float smoothRoll = 0, smoothPitch = 0, smoothYaw = 0;

// accel LPF
float ax_f=0, ay_f=0, az_f=0;

// gyro bias
float gyroBiasX=0, gyroBiasY=0, gyroBiasZ=0;
float temperature;

// ================= ESC (BLDC) =================
Servo esc;
const int escPin = 25;

int escMin = 1000;
int escMax = 2000;
int baseThrottle = 1150;
int escThrottle = 1250;
float disturbanceThreshold = 2.0;

// ================= LORA =================
#define LORA_SS   18
#define LORA_RST  14
#define LORA_DIO0 26

// ================= MPU INIT =================
void initMPU() {
  Wire.begin(21, 22);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08); // ±500 dps
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x08); // ±4g
  Wire.endTransmission();

  Serial.println("MPU initialized");
}

// ================= GYRO CAL =================
void calibrateGyro() {
  Serial.println("Calibrating gyro...");
  float sx=0, sy=0, sz=0;

  for(int i=0;i<500;i++){
    float ax,ay,az,gx,gy,gz,t;
    readMPU(ax,ay,az,gx,gy,gz,t);
    sx+=gx; sy+=gy; sz+=gz;
    delay(5);
  }

  gyroBiasX = sx/500.0;
  gyroBiasY = sy/500.0;
  gyroBiasZ = sz/500.0;
  Serial.println("Calibration done");
}

// ================= READ MPU =================
void readMPU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz,
             float &tempC) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  int16_t rawTemp = (Wire.read() << 8) | Wire.read();
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = rawAx / 8192.0;
  ay = rawAy / 8192.0;
  az = rawAz / 8192.0;

  gx = rawGx / 65.5 - gyroBiasX;
  gy = rawGy / 65.5 - gyroBiasY;
  gz = rawGz / 65.5 - gyroBiasZ;

  tempC = (rawTemp / 340.0) + 36.53;
}

// ================= FILESYSTEM =================
void initLittleFS() { LittleFS.begin(); }

// ================= WIFI =================
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(1000);
  Serial.println(WiFi.localIP());
}

// ================= LORA INIT =================
void initLoRa() {
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa failed");
    while (1);
  }
  Serial.println("LoRa TX ready");
}

// ================= GYRO JSON =================
String getGyroReadings(){
  float ax, ay, az, gx, gy, gz, t;
  readMPU(ax, ay, az, gx, gy, gz, t);

  ax_f = 0.8*ax_f + 0.2*ax;
  ay_f = 0.8*ay_f + 0.2*ay;
  az_f = 0.8*az_f + 0.2*az;

  unsigned long now = micros();
  float dt = (now - lastIMUTime)*1e-6;
  lastIMUTime = now;
  if(dt<=0 || dt>0.2) dt=0.01;

  filter.updateIMU(gx*DEG_TO_RAD, gy*DEG_TO_RAD, gz*DEG_TO_RAD,
                   ax_f, ay_f, az_f);

  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();

  smoothRoll  = 0.97*smoothRoll  + 0.03*roll;
  smoothPitch = 0.97*smoothPitch + 0.03*pitch;
  smoothYaw   = 0.995*smoothYaw  + 0.005*yaw;

  // ================= BLDC CONTROL =================
  // ================= CONTROL =================


// Threshold check
// ================= BLDC CONTROL =================

// Calculate error
error = smoothRoll - targetAngle;



// Check disturbance
bool disturbed = abs(error) > disturbanceThreshold;

// ----- P CONTROLLER -----
float Kp = 25.0;   // start small (3–8 range safe)

// Control output
escThrottle = baseThrottle + (Kp * error);

// Safety limits (VERY IMPORTANT)
escThrottle = constrain(escThrottle, 1100, 1700);

// Send to ESC
esc.writeMicroseconds(escThrottle);



  readings["gyroX"] = String(smoothRoll, 2);
  readings["gyroY"] = String(smoothPitch, 2);
  readings["gyroZ"] = String(smoothYaw, 2);
  readings["servo"] = escThrottle;

  String payload = JSON.stringify(readings);

  // ===== LoRa TX =====
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();

// ================= SERIAL DEBUG =================
Serial.println("--------------- IMU CONTROL DATA ---------------");
Serial.print("Raw Roll: "); Serial.println(roll, 2);
Serial.print("Smoothed Roll: "); Serial.println(smoothRoll, 2);
Serial.print("Pitch: "); Serial.println(smoothPitch, 2);
Serial.print("Yaw: "); Serial.println(smoothYaw, 2);

Serial.print("Target Angle: "); Serial.println(targetAngle, 2);
Serial.print("Error: "); Serial.println(error, 2);

Serial.print("Disturbance Threshold: "); Serial.println(disturbanceThreshold);
Serial.print("Disturbed? : "); Serial.println(disturbed ? "YES" : "NO");

Serial.print("Base Throttle: "); Serial.println(baseThrottle);
Serial.print("ESC Output (µs): "); Serial.println(escThrottle);
Serial.println("------------------------------------------------\n");

  return payload;
}

// ================= ACC JSON =================
String getAccReadings() {
  float ax, ay, az, gx, gy, gz, t;
  readMPU(ax, ay, az, gx, gy, gz, t);

  readings["accX"] = String(ax, 2);
  readings["accY"] = String(ay, 2);
  readings["accZ"] = String(az, 2);

  return JSON.stringify(readings);
}

// ================= TEMP =================
String getTemperature(){
  float ax, ay, az, gx, gy, gz, t;
  readMPU(ax, ay, az, gx, gy, gz, t);
  temperature = t;
  return String(temperature, 2);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  initWiFi();
  initLittleFS();
  initMPU();
  initLoRa();

  calibrateGyro();
  filter.begin(100);

  esc.attach(escPin, 1000, 2000);
  esc.writeMicroseconds(1000);
  delay(5000);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    smoothRoll = smoothPitch = smoothYaw = 0;
    filter.begin(100);
    request->send(200, "text/plain", "OK");
  });

  events.onConnect([](AsyncEventSourceClient *client){
    client->send("hello!", NULL, millis(), 10000);
  });

  server.addHandler(&events);
  server.begin();
}

// ================= LOOP =================
void loop() {
  if ((millis() - lastTime) > gyroDelay) {
    events.send(getGyroReadings().c_str(),"gyro_readings",millis());
    lastTime = millis();
  }

  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    events.send(getAccReadings().c_str(),"accelerometer_readings",millis());
    lastTimeAcc = millis();
  }

  if ((millis() - lastTimeTemperature) > temperatureDelay) {
    events.send(getTemperature().c_str(),"temperature_reading",millis());
    lastTimeTemperature = millis();
  }
}
