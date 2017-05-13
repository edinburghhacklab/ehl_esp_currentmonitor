#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

extern "C" {
#include "user_interface.h"
}

#include "config.h"

#define MAX_PAYLOAD_LENGTH 10

char my_id[40];
char mqtt_id[24];

WiFiClient client;
PubSubClient mqtt(client);

unsigned long last_read = 0;

void callback(const MQTT::Publish& pub)
{
  mqtt.publish("meta/mqtt-agents/reply", my_id);
}

double read_irms_current()
{
  const int sample_count = 1000;
  static double offsetI = 0;
  double Irms;
  double sumI = 0;

  for (unsigned int n = 0; n < sample_count; n++)
  {
    double sampleI;
    double filteredI;
    double sqI;

    sampleI = analogRead(A0);

    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
    // then subtract this - signal is now centered on 0 counts.
    offsetI = (offsetI + (sampleI-offsetI)/1024);
    filteredI = sampleI - offsetI;

    // Root-mean-square method current
    // 1) square current values
    sqI = filteredI * filteredI;
    // 2) sum
    sumI += sqI;
  }

  Irms = fudge_factor * amps_per_volt * sqrt(sumI / sample_count) / 1024;
  return Irms;
}

float read_current()
{
  const int reading_count = 70;
  const int adc_samples = 4;

  int values[reading_count];
  int min_val = 1024 * adc_samples;
  int max_val = 0;

  for (int a=0; a<reading_count; a++) {
    values[a] = 0;
    for (int i = 0; i < adc_samples; i++) {
      values[a] += analogRead(A0);
    }
  }

  for (int i=0; i<reading_count; i++) {
    if (values[i] > max_val) {
      max_val = values[i];
    }
    if (values[i] < min_val) {
      min_val = values[i];
    }
  }

  float diff = (float) (max_val - min_val) / adc_samples;
  float current = fudge_factor * amps_per_volt * diff / 1024.0 / 2.828;
  return current;
}

void setup()
{
  snprintf(my_id, sizeof(my_id), hostname_template, ESP.getChipId());
  snprintf(mqtt_id, sizeof(mqtt_id), "%08x", ESP.getChipId());

  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.print(my_id);
  Serial.print(" ");
  Serial.println(ESP.getSketchMD5());

  // run the ADC calculations a few times to stabilise the low-pass filter
  for (int i; i<10; i++) {
    read_irms_current();
    yield();
  }

  if (ota_enabled) {
    Serial.println("Enabling OTA updates");
    ArduinoOTA.setPort(8266);
    ArduinoOTA.setHostname(my_id);
    if (strlen(ota_password) > 0) {
      Serial.print("OTA password: ");
      Serial.println(ota_password);
      ArduinoOTA.setPassword(ota_password);
    }
    ArduinoOTA.onStart([]() {
      Serial.println("OTA Start");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nOTA End");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("OTA Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
  }

  wifi_station_set_hostname(my_id);
  WiFi.mode(WIFI_STA);

  mqtt.set_server(mqtt_server, mqtt_port);
  mqtt.set_callback(callback);
}

void loop()
{

  if (ota_enabled) {
    ArduinoOTA.handle();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.print(ssid);
    Serial.println("...");
    WiFi.begin(ssid, password);

    unsigned long begin_started = millis();
    while (WiFi.status() != WL_CONNECTED) {
      delay(10);
      if (millis() - begin_started > 60000) {
        ESP.restart();
      }
    }
    Serial.println("WiFi connected");
  }

  if (!mqtt.connected()) {
    if (mqtt.connect(mqtt_id)) {
      Serial.println("MQTT connected");
      mqtt.subscribe("meta/mqtt-agents/poll");
    } else {
      Serial.println("MQTT connection failed");
      delay(2000);
      return;
    }
  }

  mqtt.loop();

  if (last_read == 0 || millis() - last_read > read_interval) {
    last_read = millis();
    char mqtt_payload[MAX_PAYLOAD_LENGTH];
    double current = read_irms_current();
    dtostrf(current, 0, 2, mqtt_payload);
    Serial.print(mqtt_topic);
    Serial.print(" ");
    Serial.println(mqtt_payload);
    mqtt.publish(mqtt_topic, (const uint8_t*)mqtt_payload, strlen(mqtt_payload), true);
  }

}
