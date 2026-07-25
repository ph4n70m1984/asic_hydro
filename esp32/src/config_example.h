#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG_ENABLED 1
#define DEBUG_LEVEL 3

// Wi-Fi
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT Broker
const char* mqtt_server = "192.168.1.100";
const int mqtt_port = 1883;
const char* mqtt_user = "mqtt_user";
const char* mqtt_pass = "mqtt_password";

// Web OTA Auth
const char* http_username = "admin";
const char* http_password = "admin_password";
const char* ota_hostname = "asic-hydro-esp32";

#endif