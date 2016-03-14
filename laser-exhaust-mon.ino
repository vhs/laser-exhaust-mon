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

#define BMP180_A_SDA 3
#define BMP180_A_SCL 4
#define BMP180_B_SDA 6
#define BMP180_B_SCL 7



WiFiClient client;

static const char* k_mqttbroker = "172.16.0.161";
PubSubClient g_pubSubClient(client);


unsigned long g_lastMillisUpdate;

#define MQTT_TOPIC "cook/laser/exhaust"


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

struct measurement {
  double temperature;
  double pressure;
  bool valid;
};

measurement getMeasurement(unsigned char sda, unsigned char scl) {
  char status;
  measurement m;
  m.valid = false;

  Serial.println("D-1");
  
  SFE_BMP180 pressure;
  pressure.begin(sda, scl);

  Serial.println("D-2");

  status = pressure.startTemperature();
  if (status == 0) return m;
  delay(status);

  Serial.println("D-3");

  status = pressure.getTemperature(m.temperature);
  if (status == 0) return m;

  Serial.println("D-4");

  status = pressure.startPressure(3);
  if (status == 0) return m;
  delay(status);

  Serial.println("D-5");

  status = pressure.getPressure(m.pressure, m.temperature);
  if (status == 0) return m;

  Serial.println("D-6");

  m.valid = true;
  return m;
}

void loop(void)
{
	delay(5000);

  measurement m1 = getMeasurement(BMP180_A_SDA, BMP180_A_SCL);
  measurement m2 = getMeasurement(BMP180_B_SDA, BMP180_B_SCL);

  Serial.println("D-6.5");

  static ArduinoJson::StaticJsonBuffer<256> jsonBuf;
  Serial.println("D-6.75");
  ArduinoJson::JsonObject& obj = jsonBuf.createObject();

  Serial.println("D-7");

  obj["uptime"] = millis();

  if (m1.valid) {
    obj["m1_t"] = m1.temperature;
    obj["m1_p"] = m1.pressure;
  }

  if (m2.valid) {
    obj["m2_t"] = m2.temperature;
    obj["m2_p"] = m2.pressure;
  }

  Serial.println("D-8");

  reconnect();

  Serial.println("D-9");
  
  String str;
  obj.printTo(str);
  Serial.println(str);

  Serial.println("D-10");

  bool ret = g_pubSubClient.publish(MQTT_TOPIC, str.c_str());
  if (!ret)
  {
    Serial.println("Failed to publish");
  }
}


