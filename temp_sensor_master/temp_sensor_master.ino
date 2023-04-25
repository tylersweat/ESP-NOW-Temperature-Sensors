#include <esp_now.h>
#include <WiFi.h>
#include "time.h"
#include "sntp.h"

#define CHANNEL 0
#define ADC_PIN 2
#define ADC_SCALE 0.84

const char* ssid       = ":)";
const char* password   = "8018006622";

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const char* time_zone = PSTR("MST7MDT,M3.2.0,M11.1.0");  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)
time_t now;
bool time_available_flag = false;

void time_available_cb(struct timeval *t) {
    time_available_flag = true;
}

typedef struct struct_message {
    time_t epoch;
    float temp;
} struct_message;

struct_message temperature_data;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 1000;        // Interval at which to publish sensor readings

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    memcpy(&temperature_data, data, sizeof(temperature_data));
    Serial.printf("%s,%d,%.4f\n", macStr, temperature_data.epoch, temperature_data.temp);
}

float GetTemperature() {
    // Serial.printf("ADC: %d \n", analogRead(ADC_PIN));
    // Adjust for offset in ADC reference voltage.
    float voltage = (analogRead(ADC_PIN) / 4095.0) * 3.3 * ADC_SCALE;
    // Serial.printf("Voltage: %4.4f \n", voltage);
    float temperature = (voltage - 0.500) / 0.010; // Voltage has a 500mV offset. 10mV / C
    return temperature;
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESPNow Temperature Senser Master Node");

    Serial.println(sizeof(struct_message));
    Serial.println(sizeof(time_t));
    Serial.println(sizeof(float));

    // Get time sync.
    sntp_set_time_sync_notification_cb(time_available_cb);
    configTzTime(time_zone, ntpServer1, ntpServer2);
    //connect to WiFi
    Serial.printf("Connecting to %s ", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" CONNECTED");

    while(!time_available_flag) {
        delay(500);
    }

    // Set device to STA mode.
    WiFi.mode(WIFI_MODE_STA);
    // Init ESPNow.
    InitESPNow();
    // Register received data CB.
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("BSSID,Epoch,Temperature");
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        // Save the last time a new reading was published
        previousMillis = currentMillis;
        //Set values to send
        time(&now);
        Serial.printf("master,%d,%.4f\n", now, GetTemperature());
    }
}