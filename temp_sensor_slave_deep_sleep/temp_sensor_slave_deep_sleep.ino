#include <esp_now.h>
#include <WiFi.h>
#include "esp_adc_cal.h"
#include "time.h"
#include "sntp.h"

const uint32_t SLEEP_DURATION = 5 * 1000000; // µs

#define CHANNEL 0
#define ADC_PIN 2

#define BOARD2

#ifdef BOARD1 
#define ADC_SCALE 0.84
#endif
#ifdef BOARD2 
#define ADC_SCALE 0.87
#endif
#ifdef BOARD3 
#define ADC_SCALE 0.85
#endif
#ifdef BOARD4
#define ADC_SCALE 0.80
#endif

// const char* ssid       = ":)";
// const char* password   = "8018006622";

// const char* ntpServer1 = "pool.ntp.org";
// const char* ntpServer2 = "time.nist.gov";
// const char* time_zone = PSTR("MST7MDT,M3.2.0,M11.1.0");  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)
// time_t now;
// bool time_available_flag = false;

// void time_available_cb(struct timeval *t) {
//     time_available_flag = true;
// }

typedef struct struct_message {
    time_t epoch;
    float temp;
} struct_message;

struct_message temperature_data;
uint8_t masterAddress[] = {0x34, 0x85, 0x18, 0x23, 0x9D, 0xCC};

// unsigned long previousMillis = 0;   // Stores last time temperature was published
// const long interval = 1000;        // Interval at which to publish sensor readings

unsigned int seqNum = 0;

// Init ESP Now with fallback
void InitESPNow() {
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
        Serial.println("ESPNow Init Success");
    }
    else {
        Serial.println("ESPNow Init Failed");
        // Restart ESP32.
        ESP.restart();
    }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    // Serial.print("Last Packet Sent to: "); Serial.println(macStr);
    // Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
    Serial.println("ESPNow Temperature Senser Slave Node");

    // // Get time sync.
    // sntp_set_time_sync_notification_cb(time_available_cb);
    // configTzTime(time_zone, ntpServer1, ntpServer2);
    // //connect to WiFi
    // Serial.printf("Connecting to %s ", ssid);
    // WiFi.begin(ssid, password);
    // while (WiFi.status() != WL_CONNECTED) {
    //     delay(500);
    //     Serial.print(".");
    // }
    // Serial.println(" CONNECTED");

    // while(!time_available_flag) {
    //     delay(500);
    // }

    // Set device to STA mode.
    WiFi.mode(WIFI_MODE_STA);
    // Init ESPNow.
    InitESPNow();
    // Register sent data CB.
    esp_now_register_send_cb(OnDataSent);
    // Register peer.
    esp_now_peer_info_t masterInfo;
    memcpy(masterInfo.peer_addr, masterAddress, 6);
    masterInfo.encrypt = false;
    if (esp_now_add_peer(&masterInfo) != ESP_OK) {
        Serial.println("failed to add master as peer");
        return;
    }
}

void loop() {
    // temperature_data.epoch = 0;
    temperature_data.temp = GetTemperature();
    //Send message via ESP-NOW
    esp_err_t result = esp_now_send(masterAddress, (uint8_t *) &temperature_data, sizeof(temperature_data));
    if (result == ESP_OK) {
        Serial.println("Sent with success");
    }
    else {
        Serial.println("Error sending the data");
    }

    Serial.printf("microseconds to send: %d\n", micros());

    esp_sleep_enable_timer_wakeup(SLEEP_DURATION);
    esp_deep_sleep_start();
}