#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const uint32_t SLEEP_DURATION = 10 * 1000000; // µs

#define ADC_PIN 2

#define BOARD4

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

//set up to connect to an existing network (e.g. mobile hotspot from laptop that will run the python code)
const char* ssid = ":)";
const char* password = "8018006622";
const char* host = "192.168.1.152";
WiFiUDP Udp;
unsigned int localUdpPort = 4210;  //  port to listen on

float GetTemperature() {
    // Serial.printf("ADC: %d \n", analogRead(ADC_PIN));
    // Adjust for offset in ADC reference voltage.
    float voltage = (analogRead(ADC_PIN) / 4095.0) * 3.3 * ADC_SCALE;
    // Serial.printf("Voltage: %4.4f \n", voltage);
    float temperature = (voltage - 0.500) / 0.010; // Voltage has a 500mV offset. 10mV / C
    return temperature;
}


void setup()
{
  int status = WL_IDLE_STATUS;
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to wifi");
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

}

void loop()
{
    // once we know where we got the inital packet from, send data back to that IP address and port
    Udp.beginPacket(host, Udp.remotePort());
    // Just test touch pin - Touch0 is T0 which is on GPIO 4.
    Udp.printf(String(GetTemperature()).c_str(),2);
    Udp.endPacket();
    Serial.printf("microseconds to send: %d\n", micros());

    delay(10);

    esp_sleep_enable_timer_wakeup(SLEEP_DURATION);
    esp_deep_sleep_start();
}

