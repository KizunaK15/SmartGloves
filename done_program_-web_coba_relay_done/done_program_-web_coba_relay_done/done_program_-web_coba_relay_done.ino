#include <HTTPClient.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <MAX3010x.h>
#include "filters.h"
#include <WiFi.h>

const char* ssid = "ESP32";
const char* password = "kiaaaaaa";


BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool newData = false;

// Sensor and filtering parameters
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;
const float kEdgeThreshold = -2000.0;

const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

// Sensor pins
#define myoware A0

//Output pins
#define rjempol 13
#define rtelunjuk 12
#define rtengah 14
#define rkia 27
#define rkelingking 26
#define rpwm 33
#define lpwm 25


// BLE UUIDs
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Muscle sensor and pulse oximetry variables
uint32_t nilai_otot = 0;
float nilai_spo2 = 0.0;
int nilai_bpm = 0;

// BLE characteristic for receiving data
int jempol, telunjuk, tengah, kia, kelingking, pump, vacum, mtor, pwm;
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() == 12) { // Ensure data length is correct
        jempol = value[0];
        telunjuk = value[1];
        tengah = value[2];
        kia = value[3];
        kelingking = value[4];
        pump = value[5];
        vacum = value[6];
        mtor = value[7];
        memcpy(&pwm, value.data() + 8, sizeof(pwm)); // Copying 4 bytes of pwm to int variable

        newData = true; // Indicate that new data has been received
      }
    }
};

void setup() {
  Serial.begin(115200);
  pinMode(myoware, INPUT);
  pinMode (rjempol, OUTPUT);
  pinMode (rtelunjuk, OUTPUT);
  pinMode (rtengah, OUTPUT);
  pinMode (rkia, OUTPUT);
  pinMode (rkelingking, OUTPUT);
  pinMode (rpwm, OUTPUT);
  pinMode (lpwm, OUTPUT);

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

  BLEDevice::init("ESP32_BLE_Server");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Declare buffer
  uint8_t buffer[12] = {0};

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("Characteristic defined! Now you can read/write it on your phone!");
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

// SpO2 calibration factors
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamps and state variables
long last_heartbeat = 0;
long finger_timestamp = 0;
bool finger_detected = false;
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 10000; // 10 seconds

void loop() {
  // Sensor reading and processing

  float otot = analogRead(myoware);
  //
  //  delay(5); // Adjust delay as needed
  if (deviceConnected) {
    auto sample = sensor.readSample(1000);
    float current_value_red = sample.red;
    float current_value_ir = sample.ir;

    // Print muscle sensor value continuously
    Serial.print("Muscle Sensor Value: ");
    Serial.println(otot);

    // Detect Finger using raw sensor value
    if (sample.red > kFingerThreshold) {
      if (millis() - finger_timestamp > kFingerCooldownMs) {
        finger_detected = true;
      }
    } else {
      // Reset values if the finger is removed
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
      current_value_red = low_pass_filter_red.process(current_value_red);
      current_value_ir = low_pass_filter_ir.process(current_value_ir);

      // Statistics for pulse oximetry
      stat_red.process(current_value_red);
      stat_ir.process(current_value_ir);

      // Heartbeat detection using value for red LED
      float current_value = high_pass_filter.process(current_value_red);
      float current_diff = differentiator.process(current_value);

      if (!isnan(current_diff) && !isnan(last_diff)) {
        // Detect Heartbeat - Zero-Crossing
        if (last_diff > 0 && current_diff < 0) {
          crossed = true;
          crossed_time = millis();
        }

        if (current_diff > 0) {
          crossed = false;
        }

        // Detect Heartbeat - Falling Edge Threshold
        if (crossed && current_diff < kEdgeThreshold) {
          if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
            // Show Results
            int bpm = 60000 / (crossed_time - last_heartbeat);
            float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
            float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
            float r = rred / rir;
            float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;

            if (bpm > 50 && bpm < 250) {
              if (kEnableAveraging) {
                int average_bpm = averager_bpm.process(bpm);
                int average_r = averager_r.process(r);
                int average_spo2 = averager_spo2.process(spo2);

                if (averager_bpm.count() >= kSampleThreshold) {
                  Serial.print("Time (ms): ");
                  Serial.println(millis());
                  Serial.print("Heart Rate (avg, bpm): ");
                  Serial.println(average_bpm);
                  Serial.print("R-Value (avg): ");
                  Serial.println(average_r);
                  Serial.print("SpO2 (avg, %): ");
                  Serial.println(average_spo2);
                  //
                  //                nilai_spo2 = average_spo2;
                  //                nilai_bpm = average_bpm;
                }
              } else {
                Serial.print("Time (ms): ");
                Serial.println(millis());
                Serial.print("Heart Rate (current, bpm): ");
                Serial.println(bpm);
                Serial.print("R-Value (current): ");
                Serial.println(r);
                Serial.print("SpO2 (current, %): ");
                Serial.println(spo2);

                nilai_spo2 = spo2;
                nilai_bpm = bpm;
              }
            }

            // Reset statistic
            stat_red.reset();
            stat_ir.reset();
          }

          crossed = false;
          last_heartbeat = crossed_time;
        }
      }

      last_diff = current_diff;
    }
    
    nilai_otot = otot;
    // Send sensor data to BLE client
    uint8_t buffer[12];


    memcpy(buffer, &nilai_otot, sizeof(nilai_otot));
    memcpy(buffer + 4, &nilai_spo2, sizeof(nilai_spo2));
    memcpy(buffer + 8, &nilai_bpm, sizeof(nilai_bpm));
    pCharacteristic->setValue(buffer, sizeof(buffer));
    pCharacteristic->notify();

    // Send data to API every 10 seconds
    if (millis() - lastSendTime > sendInterval) {
      sendDataToAPI(nilai_bpm, nilai_spo2, nilai_otot);
      lastSendTime = millis();
    }
  }
  if (newData) {
    if (jempol == 0) {
      digitalWrite (rjempol, HIGH);
      Serial.print ("Jempol On, ");
    }
    else {
      digitalWrite (rjempol, LOW);
      Serial.print ("Jempol Off, ");
    }
    if (telunjuk == 0) {
      digitalWrite (rtelunjuk, HIGH);
      Serial.print ("Telunjuk On, ");
    }
    else {
      digitalWrite (rtelunjuk, LOW);
      Serial.print ("Telunjuk Off, ");
    }
    if (tengah == 0) {
      digitalWrite (rtengah, HIGH);
      Serial.print ("Tengah On, ");
    }
    else {
      digitalWrite (rtengah, LOW);
      Serial.print ("Tengah Off, ");
    }
    if (kia == 0) {
      digitalWrite (rkia, HIGH);
      Serial.print ("Kia On, ");
    }
    else {
      digitalWrite (rkia, LOW);
      Serial.print ("Kia Off, ");
    }
    if (kelingking == 0) {
      digitalWrite (rkelingking, HIGH);
      Serial.print ("Kelingking On, ");
    }
    else {
      digitalWrite (rkelingking, LOW);
      Serial.println ("Kelingking Off, ");
    }
    if ( pump == 1 && vacum == 1) {
      analogWrite (rpwm, 0);
      analogWrite (lpwm, 0);
    }
    else if (pump == 0 && vacum == 0) {
      analogWrite (rpwm, 0);
      analogWrite (lpwm, 0);
    }
    else if (pump == 0 && vacum == 1) {
      pump_on();
    }
    else if (pump == 1 && vacum == 0) {
      vacum_on();
    }
    newData = false; // Reset newData flag
  }
  delay(5);
}

void sendDataToAPI(float bpm, float spo2, float otot) {
  HTTPClient http;
  String url = "http://smarthandgloves.my.id/api/data?id_alat=alat555&action=add";
  url += "&hr=" + String((int)bpm);
  url += "&ok=" + String((int)spo2);
  url += "&ow=" + String((int)otot);

  http.begin(url); // Initialize connection
  int httpResponseCode = http.GET(); // Send GET request

  if (httpResponseCode > 0) {
    String response = http.getString(); // Get response from server
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println("Response: " + response);
  } else {
    // Serial.println("Error code: " + String(httpResponseCode));
  }

  http.end(); // Close connection
}


void pump_on() {
  analogWrite (rpwm, pwm);
  analogWrite (lpwm, 0);
}

void vacum_on() {
  analogWrite (rpwm, pwm);
  analogWrite (lpwm, 0);
  digitalWrite (rjempol, !LOW);
  digitalWrite (rtelunjuk, !LOW);
  digitalWrite (rtengah, !LOW);
  digitalWrite (rkia, !LOW);
  digitalWrite (rkelingking, !LOW);
}

void auto_gerak() {
  if (mtor == 0 && pwm < 25) {
    pump_on();
    delay(8000);
    vacum_on();
  }
  else if (mtor == 0 && pwm > 25 && pwm < 50) {
    pump_on();
    delay(6000);
    vacum_on();
  }
  else if (mtor == 0 && pwm > 50 && pwm < 75) {
    pump_on();
    delay(4000);
    vacum_on();
  }
  else {
    pump_on();
    delay(2000);
    vacum_on();
  }
}
