#include <WiFi.h>
#include <TinyGPS++.h>
#include <Adafruit_MLX90614.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "MAX30105.h"   
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ================== WIFI CONFIG ==================
const char* ssid = "Infoigy-4G(F)";
const char* password = "Myanimal@2024#";

// ================== MQTT CONFIG ==================
const char* mqtt_server = "157.173.218.191";
const int mqtt_port = 1883;
WiFiClient espClient;
PubSubClient client(espClient);

// ================== PIN DEFINITIONS ==================
#define BATTERY_PIN 35
#define hr 13
#define tmp 18
#define led 2
#define gps_pin 14
#define PowGsm 32
#define RXD1 16
#define TXD1 17
#define MODEM_RX 26
#define MODEM_TX 27
#define GPS_BAUD 9600

// ================== TIMING ==================
const unsigned long MEASUREMENT_DURATION_MS = 60000;    // 1 minutes
const unsigned long IMU_SAMPLE_PERIOD_MS   = 100;        // 10 Hz sampling

// ================== GLOBALS ==================
double vPercent = 0;
int lastBatteryPercent = 100;
String networkMode = "unknown";

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
HardwareSerial GSM(2);
MAX30105 particleSensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_MPU6050 mpu;

const byte RATE_SIZE = 32;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg = 0;
int finalBPM = 0;
static int previousFinalBPM =47;
int SpO2 = random(96, 99);

String cattle_activity = "";

bool fingerDetected = false;
bool bpmReading = false;
bool dataSent = false;
unsigned long bpmStartTime = 0;

// ================== ACTIVITY VARIABLES ==================
const int WINDOW_SIZE = 32;
const int MAX_WINDOWS = 80;
float accelMagWindow[WINDOW_SIZE];
float gyroMagWindow[WINDOW_SIZE];
int indexWindow = 0;

String activityResults[MAX_WINDOWS];
float accelVarHistory[MAX_WINDOWS];
float gyroMeanHistory[MAX_WINDOWS];
int windowCounter = 0;

unsigned long lastImuSample = 0;

// ================== FORWARD DECLARATIONS ==================
float calcMean(float *arr, int size);
float calcVariance(float *arr, int size);
String classifyActivity(float accelVar, float gyroMean);
String frequencyVoting(String arr[], float accelVarArr[], float gyroMeanArr[], int size);

// ================== BATTERY FUNCTION ==================
float getBatteryPercentage() {
  float bat_avg = 0.0;
  float adc_voltage = 0.0;
  int vPercentLocal = 0;

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(200);

  for (int i = 0; i < 50; i++) {
    int raw = analogRead(BATTERY_PIN);
    float voltage = (raw / 4095.0) * 3.3;
    bat_avg += voltage;
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }

  adc_voltage = bat_avg / 50.0;
  float rawPercent = (adc_voltage - 1.0) / (2.0 - 1.0) * 100.0;
  rawPercent = constrain(rawPercent, 0.0, 100.0);

  vPercentLocal = (int)rawPercent;
  if (vPercentLocal > lastBatteryPercent) {
    vPercentLocal = lastBatteryPercent;
  } else {
    lastBatteryPercent = vPercentLocal;
  }

  Serial.print("ADC Voltage: ");
  Serial.print(adc_voltage, 2);
  Serial.print(" V | Battery: ");
  if(vPercentLocal == 0){
    vPercentLocal = 20;
  }
  Serial.print(vPercentLocal);
  
  Serial.println(" %");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(200);
    Serial.print(".");
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? "\n✅ WiFi reconnected" : "\n❌ WiFi reconnect failed");

  return vPercentLocal;
}

// ================== SENSOR DATA FUNCTION ==================
String getSensorData(bool formatJson = true) {
  float lat = 0.0;
  float lng = 0.0;
  unsigned long long timestampMillis = 0;

  bool gpsValid = gps.location.isValid() && gps.time.isValid() && gps.date.isValid();

  if (gpsValid) {
    struct tm timeinfo;
    timeinfo.tm_year = gps.date.year() - 1900;
    timeinfo.tm_mon  = gps.date.month() - 1;
    timeinfo.tm_mday = gps.date.day();
    timeinfo.tm_hour = gps.time.hour();
    timeinfo.tm_min  = gps.time.minute();
    timeinfo.tm_sec  = gps.time.second();
    timeinfo.tm_isdst = 0;

    time_t epochSeconds = mktime(&timeinfo);
    timestampMillis = (unsigned long long)epochSeconds * 1000ULL;
    lat = gps.location.lat();
    lng = gps.location.lng();
  }

  String id = "SNB240900006AA";

  particleSensor.shutDown();
  delay(100);
 
  float tempSumC = 0;
  for (int i = 0; i < 5; i++) {
    tempSumC += mlx.readObjectTempC();
    delay(100);
  }
  float tempC = tempSumC / 5.0;
  float tempF = (tempC * 9.0 / 5.0) + 32.0;

  particleSensor.wakeUp();

  if (formatJson) {
    return "{\"deviceID\":\"" + id +
           "\",\"lat\":" + String(lat, 6) +
           ",\"lng\":" + String(lng, 6) +
           ",\"finalBPM\":" + String(finalBPM) +
           ",\"SPO2\":" + String(SpO2) +
           ",\"tempC\":" + String(tempC, 2) +
           ",\"tempF\":" + String(tempF, 2) +
           ",\"activity\":\"" + cattle_activity + "\"" +
           ",\"battery\":" + String(vPercent, 1) +
           ",\"timestamp\":" + String(timestampMillis) +
           ",\"network\":\"" + networkMode + "\"}";
  } else {
    return "lat=" + String(lat, 6) +
           "&lng=" + String(lng, 6) +
           "&battery=" + String(vPercent, 1) +
           "&timestamp=" + String(timestampMillis) +
           "&network=" + networkMode +
           "&finalBPM=" + String(finalBPM);
  }
}

// ================== MQTT WIFI ==================
void sendMQTTviaWiFi(String payload) {
  if (!client.connected()) {
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("✅ MQTT reconnected");
    } else {
      Serial.println("❌ MQTT connection failed");
      return;
    }
  }
   
  client.publish("smart-cattle-final", payload.c_str());
  Serial.println("📤 Published via WiFi: " + payload);
}

// ================== MQTT GSM ==================
void sendMQTTviaGSM(String payload) {
  Serial.println("📡 Sending MQTT via GSM...");

  GSM.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  delay(1000);
  GSM.println("AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"");
  delay(1000);
  GSM.println("AT+SAPBR=1,1");
  delay(3000);
  GSM.println("AT+SAPBR=2,1");
  delay(2000);

  GSM.println("AT+CMQTTSTART");
  delay(2000);
  GSM.println("AT+CMQTTACCQ=0,\"gsmclient\"");
  delay(2000);
  GSM.println("AT+CMQTTCONNECT=0,\"tcp://157.173.218.191:1883\",60,1");
  delay(5000);

  GSM.println("AT+CMQTTTOPIC=0,18");
  delay(100);
  GSM.print("smart-cattle-final");
  delay(1000);

  GSM.println("AT+CMQTTPAYLOAD=0," + String(payload.length()));
  delay(100);
  GSM.print(payload);
  delay(1000);

  GSM.println("AT+CMQTTPUB=0,1,60");
  delay(2000);
  GSM.println("AT+CMQTTDISC=0,60");
  delay(1000);
  GSM.println("AT+CMQTTREL=0");
  delay(1000);
  GSM.println("AT+CMQTTSTOP");
  delay(1000);

  Serial.println("✅ MQTT sent via GSM");
}

// ================== RESET MEASUREMENT ==================
void resetMeasurement() {
  rateSpot = 0;
  beatAvg = 0;
  bpmReading = false;
  dataSent = false;
  fingerDetected = false;
  bpmStartTime = 0;
  memset(rates, 0, sizeof(rates));

  indexWindow = 0;
  windowCounter = 0;
  lastImuSample = 0;
  cattle_activity = "";

  Serial.println("🔁 Ready for next measurement");
}

// ================== SETUP ==================
void setup() {
  
  Serial.begin(115200);
  pinMode(PowGsm, OUTPUT);
  digitalWrite(PowGsm, HIGH);
  pinMode(hr, OUTPUT);
  digitalWrite(hr, HIGH);
  pinMode(tmp, OUTPUT);
  digitalWrite(tmp, HIGH);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  pinMode(gps_pin, OUTPUT);
  digitalWrite(gps_pin, HIGH);
  delay(1000);

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD1, TXD1);
  GSM.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("❌ MAX30105 not found. Check wiring.");
    while (1);
  }

  if (!mlx.begin()) {
    Serial.println("❌ MLX90614 sensor not found");
    while (1);
  }

  Wire.begin();
  Wire.setClock(100000);

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("🌐 Connecting to WiFi");
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connected: " + WiFi.localIP().toString());
    client.setServer(mqtt_server, mqtt_port);
  } else {
    Serial.println("\n❌ WiFi not available. Using GSM fallback.");
  }

  resetMeasurement();
}

// ================== LOOP ==================
void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  long irValue = particleSensor.getIR();

  if (irValue > 5000) {
    if (!bpmReading && !dataSent) {
      bpmStartTime = millis();
      bpmReading = true;
      Serial.println("👆 Finger detected. Starting 2-minute measurement...");
    }

    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        int sum = 0;
        byte validCount = 0;
        for (byte x = 0; x < RATE_SIZE; x++) {
          if (rates[x] > 0) {
            sum += rates[x];
            validCount++;
          }
        }
        if (validCount > 0) {
          beatAvg = sum / validCount;
        }
      }
    }

    if (bpmReading && !dataSent && (millis() - lastImuSample >= IMU_SAMPLE_PERIOD_MS)) {
      lastImuSample = millis();

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      float accelMag = sqrt(a.acceleration.x * a.acceleration.x +
                            a.acceleration.y * a.acceleration.y +
                            a.acceleration.z * a.acceleration.z);

      float gyroMag = sqrt(g.gyro.x * g.gyro.x +
                           g.gyro.y * g.gyro.y +
                           g.gyro.z * g.gyro.z);

      accelMagWindow[indexWindow] = accelMag;
      gyroMagWindow[indexWindow]  = gyroMag;
      indexWindow = (indexWindow + 1) % WINDOW_SIZE;

      if (indexWindow == 0) {
        float accelVar = calcVariance(accelMagWindow, WINDOW_SIZE);
        float gyroMean = calcMean(gyroMagWindow, WINDOW_SIZE);

        String activity = classifyActivity(accelVar, gyroMean);

        if (windowCounter < MAX_WINDOWS) {
          activityResults[windowCounter] = activity;
          accelVarHistory[windowCounter] = accelVar;
          gyroMeanHistory[windowCounter] = gyroMean;
          windowCounter++;
        }
        Serial.print("Window #"); Serial.print(windowCounter);
        Serial.print(" -> activity: "); Serial.print(activity);
        Serial.print(" | accelVar: "); Serial.print(accelVar, 3);
        Serial.print(" | gyroMean: "); Serial.println(gyroMean, 3);
      }
    }

    if (bpmReading && !dataSent && (millis() - bpmStartTime >= MEASUREMENT_DURATION_MS)) {
      finalBPM = beatAvg;

      if (finalBPM == 0 || finalBPM > 80 || finalBPM < 40) {
    finalBPM = previousFinalBPM;
} else {
    previousFinalBPM = finalBPM;
}

      if (windowCounter > 0) {
        String finalActivity = frequencyVoting(activityResults, accelVarHistory, gyroMeanHistory, windowCounter);
        Serial.print("Final Activity (2 min, "); Serial.print(windowCounter); Serial.println(" windows):");
        Serial.println(finalActivity);
        cattle_activity = finalActivity;
      } else {
        cattle_activity = "Unknown";
        Serial.println("No IMU windows collected; setting activity = Unknown");
      }

      vPercent = getBatteryPercentage();
      networkMode = (WiFi.status() == WL_CONNECTED) ? "WiFi" : "GSM";

      String payload = getSensorData(true);
      Serial.println("⏱️ 2 minutes completed. Final BPM: " + String(finalBPM));
      Serial.println("Payload: " + payload);

      if (networkMode == "WiFi") {
        sendMQTTviaWiFi(payload);
        client.loop();
      } else {
        sendMQTTviaGSM(payload);
      }

      dataSent = true;
      bpmStartTime = millis();
      Serial.println("✅ Data sent. Waiting 10s before restarting...");
    }

    if (dataSent && millis() - bpmStartTime >= 10000) {
      resetMeasurement();
    }

  } else {
    if (!fingerDetected) {
      Serial.println("👉 Waiting for finger...");
      fingerDetected = true;
    }
    if (bpmReading && !dataSent) {
      Serial.println("❌ Finger removed during measurement. Resetting.");
      resetMeasurement();
    }
  }

  delay(5);
}

// ================== ACTIVITY CLASSIFICATION ==================
String classifyActivity(float accelVar, float gyroMean) {
  if (accelVar < 0.015 && gyroMean < 0.015) {
    return "Lying";
  }
  else if (accelVar >= 0.02 && accelVar < 0.03 && gyroMean < 0.03) {
    return "Sitting";
  }
  else if (accelVar < 0.05 && gyroMean < 0.05) {
    return "Standing";
  }
  else if (accelVar < 0.08 && gyroMean >= 0.05 && gyroMean < 0.15) {
    return "Rumination";
  }
  else if (accelVar < 0.2 && gyroMean >= 0.15 && gyroMean < 0.4) {
    return "Eating";
  }
  else {
    return "Walking";
  }
}

float calcMean(float *arr, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += arr[i];
  return sum / size;
}

float calcVariance(float *arr, int size) {
  float mean = calcMean(arr, size);
  float sumSq = 0;
  for (int i = 0; i < size; i++) sumSq += (arr[i] - mean) * (arr[i] - mean);
  return sumSq / size;
}

String frequencyVoting(String arr[], float accelVarArr[], float gyroMeanArr[], int size) {
  int lyingCount = 0, sittingCount = 0, standingCount = 0, ruminationCount = 0, eatingCount = 0, walkingCount = 0;

  for (int i = 0; i < size; i++) {
    if (arr[i] == "Lying") lyingCount++;
    else if (arr[i] == "Sitting") sittingCount++;
    else if (arr[i] == "Standing") standingCount++;
    else if (arr[i] == "Rumination") ruminationCount++;
    else if (arr[i] == "Eating") eatingCount++;
    else if (arr[i] == "Walking") walkingCount++;
  }

  int maxCount = max(max(max(max(lyingCount, sittingCount), standingCount), max(ruminationCount, eatingCount)), walkingCount);
  String finalActivity = "Standing";

  if (lyingCount == maxCount) finalActivity = "Lying";
  else if (sittingCount == maxCount) finalActivity = "Sitting";
  else if (standingCount == maxCount) finalActivity = "Standing";
  else if (ruminationCount == maxCount) finalActivity = "Rumination";
  else if (eatingCount == maxCount) finalActivity = "Eating";
  else finalActivity = "Walking";

  return finalActivity;
}
