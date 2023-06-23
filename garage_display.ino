#include <Preferences.h>
Preferences preferences;

#include <ArduinoJson.h>
#include <ArduinoJson.hpp>

/*
TODO:
- preflight for distance ranges
- set color (& other display quality based on distance)
- reduce delay
- move distance-writing OLED code to distance-reading func so we can increase freq
*/

#include <Ultrasonic.h>

#include <WiFiManager.h>
#include <strings_en.h>
#include <wm_consts_en.h>
#include <wm_strings_en.h>
#include <wm_strings_es.h>
#include <HTTPClient.h>

#include <esp_task_wdt.h>

#include <PubSubClient.h>

#include <WiFi.h>
#include "credentials.h"
WiFiClient espClient;

#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
//include <Fonts/Picopixel.h>
#include "FreeMono9pt7b.h"

unsigned long lastOLEDUpdate = 0;
unsigned long lastOffsetUpdate = 0;
int distance_offsets[] = {60, 90, 120, 150};

#define OFFSET_CONFIG_URL "http://192.168.1.2/flask/garage/config"

#define LED_PIN 27
#define LED_COLS 32
#define LED_ROWS 8
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(LED_COLS, LED_ROWS, LED_PIN,
  NEO_MATRIX_BOTTOM    + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);
bool matrixActive = true;

#define MQTT_SERVER      "192.168.1.2"
#define MQTT_SERVERPORT  1883                   // use 8883 for SSL
PubSubClient mqtt_client(espClient);
char mqtt_client_id[19];

#define HCSR04_TRIGGER 25
#define HCSR04_ECHO 26
Ultrasonic ultrasonic(HCSR04_TRIGGER, HCSR04_ECHO);
int ultrasonic_distance_int;
char ultrasonic_distance_str[10];
int ultrasonic_distance_quantized;
unsigned long lastBigChange = 0;

// LilyGo ESP32 w/ OLED--nice, but not the only thing I run this code on
#define ENABLE_OLED_DISPLAY
#ifdef ENABLE_OLED_DISPLAY
  #include <Adafruit_GFX.h>    // Core graphics library
  #include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
  #include <SPI.h>

  // pinouts from https://github.com/Xinyuan-LilyGO/TTGO-T-Display
  #define TFT_MOSI 19
  #define TFT_SCLK 18
  #define TFT_CS 5
  #define TFT_DC 16
  #define TFT_RST 23
  #define TFT_BL 4

  Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

  void setupDisplay() {
    pinMode(TFT_BL, OUTPUT);      // TTGO T-Display enable Backlight pin 4
    digitalWrite(TFT_BL, HIGH);   // T-Display turn on Backlight
    tft.init(135, 240);           // Initialize ST7789 240x135
    tft.cp437(true);
    tft.fillScreen(ST77XX_BLACK);
    tft.setRotation(1);
  }

  void padText(char text[], int amt, bool padLeft) {
    if(padLeft) {
      for(int i=0; i<(amt - strlen(text)); i++) {
        tft.print(' ');
      }
    }

    tft.print(text);

    if(!padLeft) {
      for(int i=0; i<(amt - strlen(text)); i++) {
        tft.print(' ');
      }
    }
  }
  String ipToString(IPAddress ip){
  String s="";
  for (int i=0; i<4; i++)
    s += i  ? "." + String(ip[i]) : String(ip[i]);
  return s;
}

  void refreshDisplay() {
    tft.setTextWrap(false);
    tft.setCursor(0, 0);
    tft.setTextSize(2);

    int wifiStatus = WiFi.status();
    if (wifiStatus == 0) {
      tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      padText("Wifi connecting", 30, false);
      tft.println();
    }
    else if (wifiStatus == 3) {
      tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
      padText("Wifi connected", 30, false);
      tft.println();
      
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      padText(WLAN_SSID, 30, false);
      tft.println();

      padText(const_cast<char*>(ipToString(WiFi.localIP()).c_str()), 30, false);
      tft.println();
    }
    else {
      tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
      padText("Wifi disconnected", 30, false);
      tft.println();
    }

    if (mqtt_client.connected()) {
      tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
      padText("MQTT connected", 30, false);
      tft.println();
    }
    else {
      tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
      padText("MQTT disconnected", 30, false);
      tft.println();
    }

    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.print(distance_offsets[0]);
    tft.print("/");
    tft.print(distance_offsets[1]);
    tft.print("/");
    tft.print(distance_offsets[2]);
    tft.print("/");
    tft.println(distance_offsets[3]);
  }
#else
  void setupDisplay() { return; }
  void refreshDisplay() { return; }
#endif

void setup() {
  Serial.begin(115200);

  // LED matrix
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(5);
  matrix.setTextColor(matrix.Color(0, 255, 0));
  matrix.setFont(&FreeMono9pt7b);

  // set MQTT ID
  uint64_t chipid = ESP.getEfuseMac(); // The chip ID is essentially its MAC address(length: 6 bytes).
  uint16_t chip = (uint16_t)(chipid >> 32);
  snprintf(mqtt_client_id, 19, "ESP32-%04X%08X", chip, (uint32_t)chipid);

  // initialize esp32 watchdog to 60s
  // handles wifi disassociation, among other things
  esp_task_wdt_init(60, true); //enable panic so ESP32 restarts on disconnect
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  // initialize OLED, if present
  setupDisplay();
  refreshDisplay();

  // connect to wifi
  WiFi.begin(WLAN_SSID, WLAN_PASS);

  // connect to MQTT server
  mqtt_client.setServer(MQTT_SERVER, MQTT_SERVERPORT);
  mqtt_client.setCallback(mqtt_callback);
  connect_mqtt();

  // read EEPROM offsets
  preferences.begin("garage", false);
  distance_offsets[0] = preferences.getInt("dist0", 60);
  distance_offsets[1] = preferences.getInt("dist1", 90);
  distance_offsets[2] = preferences.getInt("dist2", 120);
  distance_offsets[3] = preferences.getInt("dist3", 150);
}

void loop() {
  connect_mqtt(); // returns immediately if connected

  // handle mqtt messages
  mqtt_client.loop();

  // read ultrasonic sensor
  bool bigDistanceChange = readDistance();
  if (bigDistanceChange) {
    lastBigChange = millis();
  }

  // only keep the matrix turned on for 120s after a major distance change is detected
  if ((millis() - lastBigChange) < 120000) {
    matrixActive = true;
  }
  else if (matrixActive) {
    disableMatrix();
  }

  if ((!matrixActive) || (millis() - lastOLEDUpdate > 15000)) {
    refreshDisplay();
    lastOLEDUpdate = millis();
  }

  if ((millis() > 5000) && ((lastOffsetUpdate == 0) || (millis() - lastOffsetUpdate > 60000))) { // check offset config 1x/minute
    updateOffsets();
    lastOffsetUpdate = millis();
  }

  publishDistanceMQTT();
}

void updateOffsets() {
  Serial.println("attempting to update offsets");
  if (WiFi.status() == 3) {

    HTTPClient http;
    http.begin(OFFSET_CONFIG_URL);
    
    int httpResponseCode = http.GET();
    
    if (httpResponseCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);

      DynamicJsonDocument doc(1024);
      deserializeJson(doc, http.getString().c_str());
      for(int i=0; i<4; i++) {
        distance_offsets[i] = doc[i];
        Serial.print("distance offset ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(distance_offsets[i]);

        char prefkey[20] = "dist";
        char offset[2];
        itoa(i, offset, 10);
        strcat(prefkey, offset);
        if (preferences.getInt(prefkey) != distance_offsets[i]) {
          Serial.print("putting offset ");
          Serial.print(distance_offsets[i]);
          Serial.print(" into preference ");
          Serial.println(prefkey);
          preferences.putInt(prefkey, distance_offsets[i]);
        } 
      }
    }
    else {
      Serial.print("HTTP error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
  }
}

void publishDistanceMQTT() {
  if (mqtt_client.connected()) {
    esp_task_wdt_reset(); // all is well if we're connected to MQTT server
    mqtt_client.publish("garage/distance", ultrasonic_distance_str);
  }
}

// unused, we only publish data, not watch for it
void mqtt_callback(char* topic, byte* payload, unsigned int length) {   // callback includes topic and payload ( from which (topic) the payload is comming)
  //mqtt_client.publish("outTopic", "LED turned OFF");
}

void connect_mqtt()
{
  if (!mqtt_client.connected())
  {
    mqtt_client.connect(mqtt_client_id);
  }
}

bool readDistance() {
  bool out = false;
  ultrasonic_distance_int = ultrasonic.read();
  if ((ultrasonic_distance_int / 10) != ultrasonic_distance_quantized) {
    out = true;
  }
  ultrasonic_distance_quantized = ultrasonic_distance_int / 10;
  itoa(ultrasonic_distance_int, ultrasonic_distance_str, 10);

  if (matrixActive) {
    refreshMatrix();
  }

  int16_t  x1, y1;
  uint16_t w, h;
  tft.getTextBounds(ultrasonic_distance_str, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(0, tft.height() - h);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print("DISTANCE ");
  tft.setTextColor(getCorrectColor(), ST77XX_BLACK);
  tft.print(ultrasonic_distance_str);
  tft.println("cm                 ");

  return out;
}

void disableMatrix() {
  matrixActive = false;
  matrix.fillScreen(0);
  matrix.show();
}

uint16_t getCorrectColor() {
  const uint16_t colors[] = {
    matrix.Color(255, 0, 0), matrix.Color(255, 255, 0), matrix.Color(0, 255, 0), matrix.Color(255, 255, 0), matrix.Color(255, 0, 0)
  };
  int correctColor = 0;
  for(int i=0; i<4; i++) {
    if (ultrasonic_distance_int > distance_offsets[i]) {
      correctColor++;
    }
  }
  return colors[correctColor];
}

void refreshMatrix() {
  matrix.fillScreen(0);    //Turn off all the LEDs
  
  // determine color based on distance
  matrix.setTextColor(getCorrectColor());

  int16_t  x1, y1;
  uint16_t w, h;
  matrix.getTextBounds(ultrasonic_distance_str, 0, LED_ROWS-1, &x1, &y1, &w, &h);
  matrix.setCursor((LED_COLS - w) / 2, LED_ROWS - 1);
  matrix.print(ultrasonic_distance_str);
  
  matrix.show();
}

String httpGETRequest(const char* serverName) {
  HTTPClient http;
    
  // Your Domain name with URL path or IP address with path
  http.begin(serverName);
  
  int httpResponseCode = http.GET();
  
  String payload = "{}"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}