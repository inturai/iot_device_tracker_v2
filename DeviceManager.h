/*
 ^ The Inturai License
 * * Copyright (c) 2024 Inturai. All Rights Reserved.
 *
 * This source code is proprietary and confidential to Inturai. 
 * Unauthorized copying, distribution, modification, or use of this code, in 
 * whole or in part, is strictly prohibited without prior written consent from 
 * Inturai.
 *
 */

#ifndef DEVICEMANAGER_H
#define DEVICEMANAGER_H

#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <SecureStorage.h>
#include <ArduinoMqttClient.h>  // ✅ migrated from PubSubClient
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include "esp_wifi.h"
#include "country.h"

#define DEBUG false

#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64
#define MAX_MQTT_SERVER_LENGTH 128
#define MAX_MQTT_USERNAME_LENGTH 64
#define MAX_MQTT_PASSWORD_LENGTH 64
#define MAX_MQTT_PORT_LENGTH 5

class DeviceManager {
private:
  SecureStorage pref;
  WiFiClient espClient;
  MqttClient mqttClient{ espClient };  // ✅ ArduinoMqttClient instance

  std::string deviceName;
  int deviceType;
  std::string deviceManufacturer;
  std::string deviceVersion;
  std::string teanantId;

  const char* ca_cert;

  struct NetworkInfo {
    std::string ssid;
    int rssi;
  };

public:
  DeviceManager(const std::string& name, int type, const std::string& manufacturer,
                const std::string& version, const char* certificate)
    : mqttClient(espClient),
      deviceName(name),
      deviceType(type),
      deviceManufacturer(manufacturer),
      deviceVersion(version),
      ca_cert(certificate) {}

  std::string getTenantId() {
    return teanantId;
  }

  int handleCommand(std::string commandString) {
    size_t first = commandString.find_first_not_of(' ');
    size_t last = commandString.find_last_not_of(' ');
    if (std::string::npos != first && std::string::npos != last)
      commandString = commandString.substr(first, (last - first + 1));
    else
      commandString = "";

    size_t colonIndex = commandString.find(':');
    if (colonIndex == std::string::npos) {
      writeResponse(400, -1, "Invalid Command Format",
                    "{\"error\":\"Command must follow the format 'command:payload'\"}");
      return -1;
    }

    int commandCode = 0;
    try {
      commandCode = std::stoi(commandString.substr(0, colonIndex));
    } catch (...) {
      commandCode = -1;
    }

    std::string payload = commandString.substr(colonIndex + 1);

    switch (commandCode) {
      case 0:
        deviceInfo();
        break;
      case 2:
        saveConfiguration(payload);
        break;
      case 3:
        retrieveConfiguration();
        break;
      case 999:
        clearConfiguration();
        break;
      default:
        writeResponse(400, commandCode, "Invalid Command", "{}");
        break;
    }

    return commandCode;
  }

  void saveConfiguration(const std::string& payload) {
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, payload.c_str());
    if (error) {
      writeResponse(400, 2, "Invalid JSON format",
                    "{\"error\":\"Invalid JSON format\"}");
      return;
    }

    pref.begin("config", false);

    std::string deviceId;
    if (doc.containsKey("device_id")) {
      deviceId = doc["device_id"].as<const char*>();
      if (!deviceId.empty()) {
        pref.putString("device_id", deviceId);
      } else {
        std::ostringstream ss;
        ss << std::hex << ESP.getEfuseMac();
        deviceId = ss.str();
      }
    } else {
      std::ostringstream ss;
      ss << std::hex << ESP.getEfuseMac();
      deviceId = ss.str();
      pref.putString("device_id", deviceId);
    }

    std::string wifiResponse = validateWiFi(doc);
    if (wifiResponse.find("saved successfully") == std::string::npos) {
      pref.end();
      writeResponse(400, 2, wifiResponse, "{}");
      return;
    }

    std::string mqttResponse = validateMQTT(doc);
    if (mqttResponse.find("saved successfully") == std::string::npos) {
      pref.end();
      writeResponse(400, 2, mqttResponse, "{}");
      return;
    }

    std::string tenantResponse = validateAndSaveTenantInfo(doc);
    if (tenantResponse.find("saved successfully") == std::string::npos) {
      pref.end();
      writeResponse(400, 2, tenantResponse, "{}");
      return;
    }

    std::string deviceConfigResponse = validateAndSaveDeviceParameter(doc);
    if (deviceConfigResponse.find("saved successfully") == std::string::npos) {
      pref.end();
      writeResponse(400, 2, deviceConfigResponse, "{}");
      return;
    }

    doc.clear();
    std::string tenantId = pref.getString("tenant_id");

    pref.putString("TOPIC_DATA", "int/iot/" + tenantId + "/devices/" + deviceId + "/data");
    pref.putString("TOPIC_HEALTH", "int/iot/" + tenantId + "/devices/" + deviceId + "/health");
    pref.end();

    if (mqttClient.connected()) mqttClient.stop();
    WiFi.disconnect();

    writeResponse(200, 2, "Device configuration saved successfully", "{}");
  }

  std::string validateAndSaveTenantInfo(StaticJsonDocument<1024>& doc) {
    if (doc.containsKey("tenant_id")) {
      std::string tenantId = doc["tenant_id"].as<const char*>();
      if (tenantId.empty()) return "Tenant ID is empty or invalid.";
      pref.putString("tenant_id", tenantId);
      return "Tenant ID saved successfully.";
    } else {
      return "Tenant ID missing in the configuration.";
    }
  }

  std::string validateAndSaveDeviceParameter(StaticJsonDocument<1024>& doc) {
    if (doc.containsKey("publish_interval")) {
      int temp = doc["publish_interval"].as<int>();
      if (temp <= 0) return "publish_interval is invalid.";
      pref.putInt("pub_int", temp);
    } else return "publish_interval missing.";

    if (doc.containsKey("scan_interval")) {
      int scan_interval = doc["scan_interval"].as<int>();
      if (scan_interval <= 0) return "scan_interval is invalid.";
      pref.putInt("scan_interval", scan_interval);
    } else return "scan_interval missing.";

    if (doc.containsKey("device_ttl")) {
      int device_ttl = doc["device_ttl"].as<int>();
      if (device_ttl <= 0) return "device_ttl is invalid.";
      pref.putInt("device_ttl", device_ttl);
    } else return "device_ttl missing.";

    if (doc.containsKey("rssi_filter")) {
      int rssi_filter = doc["rssi_filter"].as<int>();
      if (rssi_filter == 0) return "rssi_filter is invalid.";
      pref.putInt("rssi_filter", rssi_filter);
    } else return "rssi_filter missing.";

    if (doc.containsKey("geofence_radius")) {
      int geofence_radius = doc["geofence_radius"].as<int>();
      if (geofence_radius <= 0) return "geofence_radius invalid.";
      pref.putInt("geofence_radius", geofence_radius);
    } else return "geofence_radius missing.";

    if (doc.containsKey("country")) {
      std::string country = doc["country"].as<const char*>();
      if (country.empty()) {
        country = "AU";
      }
      pref.putString("country", country);
    }

    return "device parameters saved successfully.";
  }

  std::string validateWiFi(StaticJsonDocument<1024>& doc) {
    if (doc.containsKey("ssid") && doc.containsKey("ssid_password")) {
      std::string ssid = doc["ssid"].as<const char*>();
      std::string ssidPassword = doc["ssid_password"].as<const char*>();
      if (ssid.empty() || ssidPassword.empty())
        return "Wi-Fi credentials missing or invalid.";

      std::string wifiCountry = "AU";  // Default to a common zone (Zone 2)
      if (doc.containsKey("country")) {
        wifiCountry = doc["country"].as<const char*>();
      }

      // Call the helper method with the country code string
      setWifiCountryByString(wifiCountry.c_str());

      WiFi.begin(ssid.c_str(), ssidPassword.c_str());

      int timeout = 0;
      while (WiFi.status() != WL_CONNECTED && timeout < 20) {
        delay(500);
        timeout++;
      }

      if (WiFi.status() != WL_CONNECTED)
        return "Failed to connect to Wi-Fi.";

      pref.putString("ssid", ssid);
      pref.putEncryptedString("ssid_password", ssidPassword);
      return "Wi-Fi credentials saved successfully.";
    } else {
      return "Wi-Fi credentials missing in configuration.";
    }
  }

  // ✅ MIGRATED TO ArduinoMqttClient
  std::string validateMQTT(StaticJsonDocument<1024>& doc) {
    if (doc.containsKey("mqtt_server") && doc.containsKey("mqtt_username") && doc.containsKey("mqtt_password") && doc.containsKey("mqtt_port")) {

      std::string mqttServer = doc["mqtt_server"].as<const char*>();
      std::string mqttUsername = doc["mqtt_username"].as<const char*>();
      std::string mqttPassword = doc["mqtt_password"].as<const char*>();
      int mqttPort = doc["mqtt_port"].as<int>();

      if (mqttServer.empty() || mqttPort <= 0)
        return "MQTT server or port invalid.";

      mqttClient.setId("ESP32Client");
      mqttClient.setUsernamePassword(mqttUsername.c_str(), mqttPassword.c_str());

      if (mqttClient.connect(mqttServer.c_str(), mqttPort)) {
        pref.putString("mqtt_server", mqttServer);
        pref.putString("mqtt_username", mqttUsername);
        pref.putEncryptedString("mqtt_password", mqttPassword);
        pref.putInt("mqtt_port", mqttPort);
        mqttClient.stop();  // ✅ FIXED
        return "MQTT credentials saved successfully.";
      } else {
        return "Failed to connect to MQTT broker.";
      }
    } else {
      return "MQTT credentials missing.";
    }
  }

  void clearConfiguration() {
    WiFi.mode(WIFI_OFF);
    delay(3000);
    pref.begin("config", false);
    pref.clear();
    pref.end();
    writeResponse(200, 999, "Configuration cleared successfully", "{}");
  }

  void retrieveConfiguration() {
    if (DEBUG)
      Serial.println("Retrieve Configuration");
    writeResponse(403, 3, "Forbidden",
                  "{\"error\":\"Access to the configuration is forbidden.\"}");
  }

  void deviceInfo(int topCount = 15) {
    StaticJsonDocument<2048> requestBody;
    uint64_t mac = ESP.getEfuseMac();
    char macStr[13];
    snprintf(macStr, sizeof(macStr), "%012llx", mac);

    requestBody["name"] = deviceName;
    requestBody["serial_number"] = macStr;
    requestBody["model"] = ESP.getChipModel();
    requestBody["manufacturer"] = deviceManufacturer;
    requestBody["cpu_frequency_mhz"] = ESP.getCpuFreqMHz();
    requestBody["device_type"] = deviceType;
    requestBody["flash_size_mb"] = ESP.getFlashChipSize() / (1024 * 1024);
    requestBody["flash_speed_mhz"] = ESP.getFlashChipSpeed() / 1000000;

    pref.begin("config", false);
    requestBody["ssid"] = pref.getString("ssid");
    requestBody["ssid_password"] = pref.getDecryptedString("ssid_password");
    requestBody["scan_interval"] = pref.getInt("scan_interval", 6000);
    requestBody["publish_interval"] = pref.getInt("pub_int", 15000);
    requestBody["device_ttl"] = pref.getInt("device_ttl", 5000);
    requestBody["rssi_filter"] = pref.getInt("rssi_filter", -100);
    requestBody["geofence_radius"] = pref.getInt("geofence_radius", 8);
    requestBody["country"] = pref.getString("country", "AU");

    pref.end();

    WiFi.mode(WIFI_STA);
    int numNetworks = WiFi.scanNetworks();

    std::vector<int> indices(numNetworks);
    for (int i = 0; i < numNetworks; ++i) indices[i] = i;
    std::sort(indices.begin(), indices.end(),
              [&](int i, int j) {
                return WiFi.RSSI(i) > WiFi.RSSI(j);
              });

    JsonArray ssidArray = requestBody.createNestedArray("available_ssids");
    for (int i = 0; i < std::min(topCount, numNetworks); i++) {
      int sortedIndex = indices[i];
      ssidArray.add(WiFi.SSID(sortedIndex).c_str());
    }

    WiFi.scanDelete();

    std::string requestBodyStr;
    serializeJson(requestBody, requestBodyStr);
    writeResponse(200, 0, "Success", requestBodyStr);
  }

  void writeResponse(int status_code, int request_code,
                     const std::string& status_message,
                     const std::string& payload) {
    std::string json;
    json.reserve(2048);
    json += "{\"status_code\":";
    json += std::to_string(status_code);
    json += ",\"request_code\":";
    json += std::to_string(request_code);
    json += ",\"status_message\":\"";
    json += status_message;
    json += "\",\"payload\":";
    json += payload;
    json += "}";
    Serial.println(json.c_str());
  }
};

#endif  // DEVICEMANAGER_H
