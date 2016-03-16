#define MQTT_MAX_PACKET_SIZE 256

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "SFE_BMP180.h"

// Define these in the wifi_creds.hpp file
//const char ssid[] = "YOUR SSID"
//const char wifi_passwd[] = "YOUR WIFI PASSWORD"
#include "wifi_creds.h"

#define MQTT_TOPIC "cook/laser/exhaust"

#define BMP180_A_SDA 0
#define BMP180_A_SCL 2
#define BMP180_B_SDA 13
#define BMP180_B_SCL 12

struct measurement {
  double temperature;
  double pressure;
  bool valid;
};

WiFiClient client;
static const char* k_mqttbroker = "172.16.0.161";
PubSubClient g_pubSubClient(client);

void reconnect() {
  // Loop until we're reconnected
  while (!g_pubSubClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (g_pubSubClient.connect("Laser Exhaust Monitor")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(g_pubSubClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void DoWifiConnect()
{
  WiFi.begin(ssid, wifi_passwd);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup(void){
  Serial.begin(115200);

  DoWifiConnect();

  g_pubSubClient.setServer(k_mqttbroker,1883);

  reconnect();
}


measurement getMeasurement(unsigned char sda, unsigned char scl) {
  char status;
  measurement m;
  m.valid = false;

  SFE_BMP180 pressure;
  pressure.begin(sda, scl);

  status = pressure.startTemperature();
  if (status == 0) return m;
  delay(status);

  status = pressure.getTemperature(m.temperature);
  if (status == 0) return m;

  Serial.print("Temp: ");
  Serial.println(m.temperature);

  status = pressure.startPressure(3);
  if (status == 0) return m;
  delay(status);

  status = pressure.getPressure(m.pressure, m.temperature);
  if (status == 0) return m;
  Serial.print("Pressure: ");
  Serial.println(m.pressure);

  m.valid = true;
  return m;
}

void loop(void)
{
  delay(5000);

  measurement m1 = getMeasurement(BMP180_A_SDA, BMP180_A_SCL);
  measurement m2 = getMeasurement(BMP180_B_SDA, BMP180_B_SCL);

  ArduinoJson::StaticJsonBuffer<256> jsonBuf;
  ArduinoJson::JsonObject& obj = jsonBuf.createObject();

  obj["uptime"] = millis();

  if (m1.valid) {
    obj["m1_t"] = m1.temperature;
    obj["m1_p"] = m1.pressure;
  }

  if (m2.valid) {
    obj["m2_t"] = m2.temperature;
    obj["m2_p"] = m2.pressure;
  }

  reconnect();

  String str;
  obj.printTo(str);
  Serial.println(str);

  bool ret = g_pubSubClient.publish(MQTT_TOPIC, str.c_str());
  if (!ret)
  {
    Serial.println("Failed to publish");
  }
}
