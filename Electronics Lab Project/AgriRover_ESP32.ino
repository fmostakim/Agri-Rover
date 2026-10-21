#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <DHT.h>

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

const char *ssid = "ESP32CAM_SOFTAP";
const char *password = "Testing123";

// ===========================
// Rover Hardware Pins
// ===========================
const int IN1 = 12;         
const int IN2 = 15;         
const int irPin = 13;       
const int dhtPin = 14;      

#define DHTTYPE DHT11
DHT dht(dhtPin, DHTTYPE);

// API Server (Port 81)
WiFiServer apiServer(81);

int motorSpeed = 150;        
bool isForward = true;      

unsigned long previousMillis = 0;
bool isMoving = true;
const long moveTime = 2000; 
const long stopTime = 5000; 
bool actionDone = false;    

float currentTemp = 0.0;
float currentHum = 0.0;

void startCameraServer();
void setupLedFlash();

void moveRover() {
  if (isForward) {
    analogWrite(IN1, motorSpeed);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, motorSpeed);
  }
}

void stopRover() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(irPin, INPUT_PULLUP);
  
  stopRover(); 

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  
  dht.begin(); 

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;       
  config.pixel_format = PIXFORMAT_RGB565;   
  config.grab_mode = CAMERA_GRAB_LATEST;    
  config.fb_location = CAMERA_FB_IN_PSRAM;
  
  if (psramFound()) {
    config.fb_count = 2;                    
  } else {
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_camera_init(&config);
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

  WiFi.softAP(ssid, password);
  startCameraServer();
  apiServer.begin();
  
  moveRover(); 
  isMoving = true;
  previousMillis = millis(); 
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Auto-Reverse on Pole Detection
  if (digitalRead(irPin) == LOW) { 
    stopRover();
    delay(1000);            
    isForward = !isForward; 
    moveRover();
    delay(1500);            
    isMoving = true;
    previousMillis = millis(); 
  }

  // 2. Stop-and-Scan Logic
  if (isMoving) {
    if (currentMillis - previousMillis >= moveTime) {
      stopRover();           
      isMoving = false;      
      actionDone = false; 
      previousMillis = currentMillis; 
    }
  } else {
    if (!actionDone) {
      currentHum = dht.readHumidity();
      currentTemp = dht.readTemperature();
      actionDone = true; 
    }
    if (currentMillis - previousMillis >= stopTime) {
      moveRover();           
      isMoving = true;       
      previousMillis = currentMillis; 
    }
  }

  // 3. JSON API Server for Dashboard
  WiFiClient client = apiServer.available();
  if (client) {
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n' && currentLineIsBlank) {
          String dirState = isForward ? "Forwarding" : "Backwarding";
          String roverState = isMoving ? "Moving" : "Scanning";

          String json = "{";
          json += "\"direction\": \"" + dirState + "\",";
          json += "\"temperature\": " + (isnan(currentTemp) ? String("0") : String(currentTemp)) + ",";
          json += "\"humidity\": " + (isnan(currentHum) ? String("0") : String(currentHum)) + ",";
          json += "\"rover_status\": \"" + roverState + "\"";
          json += "}";

          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: application/json");
          client.println("Access-Control-Allow-Origin: *"); 
          client.println("Connection: close");
          client.println();
          client.println(json);
          break;
        }
        if (c == '\n') {
          currentLineIsBlank = true;
        } else if (c != '\r') {
          currentLineIsBlank = false;
        }
      }
    }
    delay(1);
    client.stop();
  }
  delay(10); 
}