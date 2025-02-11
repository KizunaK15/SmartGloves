#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <MAX3010x.h>
#include "filters.h"
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "ESP32";
const char* password = "kiaaaaaa";

#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "abcdef01-1234-5678-1234-56789abcdef0"
#define BPM_CHARACTERISTIC_UUID "abcdef02-1234-5678-1234-56789abcdef0"
#define SPO2_CHARACTERISTIC_UUID "abcdef03-1234-5678-1234-56789abcdef0"
#define OTOT_CHARACTERISTIC_UUID "abcdef04-1234-5678-1234-56789abcdef0"

#define Rjempol 13
#define Rtelunjuk 32
#define Rtengah 14
#define Rkia 27
#define Rkelingking 26
#define rpwm 33
#define lpwm 25

int jempol, telunjuk, tengah, kia, kelingking;
int pwm, mtor, vacum, pump;

MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

int nilai_bpm;
float nilai_spo2;
int nilai_otot;

// Buffer to store BPM data
const int kDataCount = 5;
int bpmData[kDataCount];
int bpmIndex = 0;
unsigned long lastSampleTime = 0;
unsigned long startTime = 0;
bool isSampling = false;

BLEClient* pClient;
BLERemoteService* pRemoteService;
BLERemoteCharacteristic* pRemoteCharacteristic;
BLERemoteCharacteristic* pBpmCharacteristic;
BLERemoteCharacteristic* pSpo2Characteristic;
BLERemoteCharacteristic* pOtotCharacteristic;

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pClient) {
      Serial.println("Connected to BLE Server");
    }

    void onDisconnect(BLEClient* pClient) {
      Serial.println("Disconnected from BLE Server");
    }
};

void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  String receivedData = String((char*)pData);

  int values[5] = {0};
  int index = 0;
  int start = 0;

  for (int i = 0; i < receivedData.length(); i++) {
    if (receivedData[i] == ',' || i == receivedData.length() - 1) {
      if (i == receivedData.length() - 1) i++;
      values[index] = receivedData.substring(start, i).toInt();
      start = i + 1;
      index++;
    }
  }

  if (index == 5) { // for jempol, telunjuk, tengah, kia, kelingking
    jempol = values[0];
    telunjuk = values[1];
    tengah = values[2];
    kia = values[3];
    kelingking = values[4];
  } else if (index == 4) { // for pwm, mtor, vacum, pump
    pwm = values[0];
    mtor = values[1];
    vacum = values[2];
    pump = values[3];
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(Rjempol, OUTPUT);
  pinMode(Rtelunjuk, OUTPUT);
  pinMode(Rtengah, OUTPUT);
  pinMode(Rkia, OUTPUT);
  pinMode(Rkelingking, OUTPUT);
  pinMode(rpwm, OUTPUT);
  pinMode(lpwm, OUTPUT);

  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    Serial.println("Sensor initialized");
  } else {
    Serial.println("Sensor not found");
    while (1);
  }

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");


  BLEDevice::init("ESP32_Client");
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  BLEAddress serverAddress("30:30:f9:6c:64:11");
  pClient->connect(serverAddress);

  pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.println("Failed to find service UUID");
    pClient->disconnect();
    return;
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Failed to find characteristic UUID");
    pClient->disconnect();
    return;
  }

  pRemoteCharacteristic->registerForNotify(notifyCallback);

  pBpmCharacteristic = pRemoteService->getCharacteristic(BPM_CHARACTERISTIC_UUID);
  pSpo2Characteristic = pRemoteService->getCharacteristic(SPO2_CHARACTERISTIC_UUID);
  pOtotCharacteristic = pRemoteService->getCharacteristic(OTOT_CHARACTERISTIC_UUID);
}

// Filter Instances
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

long last_heartbeat = 0;
long finger_timestamp = 0;
bool finger_detected = false;
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 10000; // 10 seconds

void loop() {
  // Read values from the sensor
  nilai_otot = analogRead(A0);
  auto sample = sensor.readSample(1000);
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;

  Serial.print("Raw Red Value: ");
  Serial.println(current_value_red);
  Serial.print("Raw IR Value: ");
  Serial.println(current_value_ir);

  // Finger detection logic
  if (sample.red > kFingerThreshold) {
    if (millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  } else {
    // Reset everything if no finger detected
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();

    finger_detected = false;
    finger_timestamp = millis();
  }

  if (finger_detected) {
    // Process the sensor data
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);

    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);

    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);

    if (!isnan(current_diff) && !isnan(last_diff)) {
      if (last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }

      if (current_diff > 0) {
        crossed = false;
      }

      if (crossed && current_diff < kEdgeThreshold) {
        if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          int bpm = 60000 / (crossed_time - last_heartbeat);
          float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
          float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
          float r = rred / rir;
          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;

          if (bpm > 50 && bpm < 250) {
            if (kEnableAveraging) {
              bpm = averager_bpm.process(bpm);
              r = averager_r.process(r);
              spo2 = averager_spo2.process(spo2);
            }

            Serial.print("Time (ms): ");
            Serial.println(millis());
            Serial.print("Heart Rate (bpm): ");
            Serial.println(bpm);
            Serial.print("R-Value: ");
            Serial.println(r);
            Serial.print("SpO2 (%%): ");
            Serial.println(spo2);

            nilai_bpm = bpm;
            nilai_spo2 = spo2;

            // Save BPM data every 2 seconds
            if (isSampling && millis() - lastSampleTime >= 2000) {
              bpmData[bpmIndex] = bpm;
              bpmIndex = (bpmIndex + 1) % kDataCount;
              lastSampleTime = millis();
            }
          }

          stat_red.reset();
          stat_ir.reset();
        }

        crossed = false;
        last_heartbeat = crossed_time;
      }
    }

    last_diff = current_diff;
  }

  // Send data to BLE characteristics
  if (pBpmCharacteristic != nullptr) {
    pBpmCharacteristic->writeValue(String(nilai_bpm).c_str());
  }

  if (pSpo2Characteristic != nullptr) {
    pSpo2Characteristic->writeValue(String(nilai_spo2).c_str());
  }

  if (pOtotCharacteristic != nullptr) {
    pOtotCharacteristic->writeValue(String(nilai_otot).c_str());
  }

  jari();

  // Manage sampling and display intervals
  static unsigned long samplingStartTime = 0;
  static unsigned long lastDisplayTime = 0;
  static bool isSampling = false;

  if (finger_detected && !isSampling) {
    isSampling = true;
    samplingStartTime = millis();
    lastSampleTime = samplingStartTime;
    lastDisplayTime = samplingStartTime;
  }

  if (isSampling) {
    if (millis() - samplingStartTime >= 10000) {
      isSampling = false;

      // Calculate and display the average BPM over 10 seconds
      float averageBPM = 0;
      int count = 0;

      for (int i = 0; i < kDataCount; i++) {
        if (bpmData[i] > 0) {
          averageBPM += bpmData[i];
          count++;
        }
      }

      if (count > 0) {
        averageBPM /= count;
      }

      Serial.print("Average BPM over 10 seconds: ");
      Serial.println(averageBPM);
    }

    // Display average BPM every 10 seconds
    if (millis() - lastDisplayTime >= 10000) {
      float averageBPM = 0;
      int count = 0;

      for (int i = 0; i < kDataCount; i++) {
        if (bpmData[i] > 0) {
          averageBPM += bpmData[i];
          count++;
        }
      }

      if (count > 0) {
        averageBPM /= count;
      }

      // Send data to API every 10 seconds
      if (millis() - lastSendTime > sendInterval) {
        sendDataToAPI(nilai_bpm, nilai_spo2, nilai_otot);
        lastSendTime = millis();
      }

      Serial.print("Average BPM every 10 seconds: ");
      Serial.println(averageBPM);

      lastDisplayTime = millis(); // Update last display time
    }
  }
}



// Memperbarui bagian sendDataToAPI dengan penggunaan POST request
void sendDataToAPI(float bpm, float spo2, float otot) {
  HTTPClient http;
  String url = "https://smarthandgloves.my.id/api/data";
  http.begin(url); // Initialize connection

  // Prepare JSON payload
  String payload = "{";
  payload += "\"id_alat\":\"alat123\",";
  payload += "\"action\":\"add\",";
  payload += "\"hr\":" + String((int)bpm) + ",";
  payload += "\"ok\":" + String((int)spo2) + ",";
  payload += "\"ow\":" + String((int)otot);
  payload += "}";

  http.addHeader("Content-Type", "application/json"); // Specify content type
  int httpResponseCode = http.POST(payload); // Send POST request with JSON payload

  if (httpResponseCode > 0) {
    String response = http.getString(); // Get response from server
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println("Response: " + response);
  } else {
    Serial.println("Error code: " + String(httpResponseCode));
  }

  http.end(); // Close connection
}


void jari() {
  // Debug print statements to see the values of the variables
  Serial.print("jempol = ");
  Serial.print(jempol);
  Serial.print(", telunjuk = ");
  Serial.print(telunjuk);
  Serial.print(", tengah = ");
  Serial.print(tengah);
  Serial.print(", kia = ");
  Serial.print(kia);
  Serial.print(", kelingking = ");
  Serial.println(kelingking);

  if (jempol == 0) {
    digitalWrite(Rjempol, HIGH);
    Serial.print(", Rjempol = On, ");
  } else {
    digitalWrite(Rjempol, LOW);
    Serial.print(", Rjempol = Off, ");
  }

  if (telunjuk == 0) {
    digitalWrite(Rtelunjuk, HIGH);
    Serial.print("Rtelunjuk = On, ");
  } else {
    digitalWrite(Rtelunjuk, LOW);
    Serial.print("Rtelunjuk = Off, ");
  }

  if (tengah == 0) {
    digitalWrite(Rtengah, HIGH);
    Serial.print("Rtengah = On, ");
  } else {
    digitalWrite(Rtengah, LOW);
    Serial.print("Rtengah = Off, ");
  }

  if (kia == 0) {
    digitalWrite(Rkia, HIGH);
    Serial.print("Rkia = On, ");
  } else {
    digitalWrite(Rkia, LOW);
    Serial.print("Rkia = Off, ");
  }

  if (kelingking == 0) {
    digitalWrite(Rkelingking, HIGH);
    Serial.println("Rkelingking = On, ");
  } else {
    digitalWrite(Rkelingking, LOW);
    Serial.println("Rkelingking = Off, ");
  }

  if (pump == 0) {
    analogWrite(rpwm, pwm);
    analogWrite(lpwm, 0);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}
