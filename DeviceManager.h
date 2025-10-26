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
#include <PubSubClient.h>  // Ensure this is included if PubSubClient is used
#include <string>          // Required for std::string
#include <vector>          // Required for std::vector
#include <algorithm>       // Required for std::sort and std::min
#include <sstream>         // For number-to-string conversion

// Define the maximum buffer sizes for various configurations
#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64
#define MAX_MQTT_SERVER_LENGTH 128
#define MAX_MQTT_USERNAME_LENGTH 64
#define MAX_MQTT_PASSWORD_LENGTH 64
#define MAX_MQTT_PORT_LENGTH 5

class DeviceManager {
private:
  // Preferences pref;  // For saving configurations in ESP32
  SecureStorage pref;
  WiFiClient espClient;
  //WiFiClientSecure espClient;

  PubSubClient mqttClient;  // MQTT client

  std::string deviceName;          // Device name (REPLACED String)
  int deviceType;                  // Device type
  std::string deviceManufacturer;  // Device manufacturer (REPLACED String)
  std::string deviceVersion;       // Device version (REPLACED String)

  //Device id and Tenant ID
  std::string teanantId;  // REPLACED String

  const char* ca_cert;

  // --- Helper structs/functions for deviceInfo ---
  // Using struct inside private scope
  struct NetworkInfo {
    std::string ssid;
    int rssi;
  };
  // --- End Helper structs/functions ---

public:
  // Constructor for initialization with name, type, manufacturer, and version of the device
  // REPLACED String parameters with const std::string&
  DeviceManager(const std::string& name, int type, const std::string& manufacturer, const std::string& version, const char* certificate)
    : mqttClient(espClient), deviceName(name), deviceType(type), deviceManufacturer(manufacturer), deviceVersion(version), ca_cert(certificate) {
    // espClient.setCACert(ca_cert);  // Set the CA certificate
  }

  // Returns Tenant ID (REPLACED String)
  std::string getTenantId() {
    return teanantId;
  }

  // Method to handle incoming commands (REPLACED String)
  int handleCommand(std::string commandString) {

    // Trim equivalent for std::string (simple implementation for this context)
    size_t first = commandString.find_first_not_of(' ');
    size_t last = commandString.find_last_not_of(' ');
    if (std::string::npos != first && std::string::npos != last) {
      commandString = commandString.substr(first, (last - first + 1));
    } else {
      commandString = "";  // Fully whitespace or empty
    }

    // Ensure command follows the 'command:payload' format
    size_t colonIndex = commandString.find(':');
    if (colonIndex == std::string::npos) {
      writeResponse(400, -1, "Invalid Command Format", "{\"error\":\"Command must follow the format 'command:payload'\"}");
      return -1;
    }

    // Extract command code and payload
    // Use std::stoi to convert part of string to int
    int commandCode = 0;
    try {
      commandCode = std::stoi(commandString.substr(0, colonIndex));
    } catch (...) {
      // Handle conversion error if needed, but in this context, 0 or default is usually fine
      commandCode = -1;
    }

    std::string payload = commandString.substr(colonIndex + 1);  // REPLACED String

    // Handle the command based on the command code
    switch (commandCode) {
      case 0:  // Get Device info
        deviceInfo();
        break;
      case 2:  // Save configuration
        saveConfiguration(payload);
        break;
      case 3:  // Retrieve saved configuration
        retrieveConfiguration();
        break;
      case 999:  // Clear configuration
        clearConfiguration();
        break;
      default:
        writeResponse(400, commandCode, "Invalid Command", "{}");
        break;
    }

    return commandCode;
  }

  // Save the configuration from the payload (REPLACED String)
  void saveConfiguration(const std::string& payload) {

    StaticJsonDocument<1024> doc;

    // Use .c_str() for deserialization
    DeserializationError error = deserializeJson(doc, payload.c_str());

    // Check if JSON deserialization was successful
    if (error) {
      writeResponse(400, 2, "Invalid JSON format", "{\"error\":\"Invalid JSON format\"}");
      return;
    }

    // Initialize Preferences
    pref.begin("config", false);

    std::string deviceId;  // REPLACED String
    if (doc.containsKey("device_id")) {
      // Use .c_str() to convert to std::string
      deviceId = doc["device_id"].as<const char*>();
      if (!deviceId.empty()) {  // Use .empty()
        pref.putString("device_id", deviceId);
      } else {
        // Generate device ID based on MAC address
        std::ostringstream ss;
        ss << std::hex << ESP.getEfuseMac();
        deviceId = ss.str();
      }
    } else {
      // Generate device ID based on MAC address
      std::ostringstream ss;
      ss << std::hex << ESP.getEfuseMac();
      deviceId = ss.str();
    }

    // Wi-Fi Credentials Validation
    std::string wifiResponse = validateWiFi(doc);                        // REPLACED String
    if (wifiResponse.find("saved successfully") == std::string::npos) {  // Use .fidata and std::string::npos
      pref.end();
      writeResponse(400, 2, wifiResponse, "{}");
      return;
    }

    // MQTT Credentials Validation (Only after Wi-Fi success)
    std::string mqttResponse = validateMQTT(doc);                        // REPLACED String
    if (mqttResponse.find("saved successfully") == std::string::npos) {  // Use .find() and std::string::npos
      pref.end();
      writeResponse(400, 2, mqttResponse, "{}");
      return;
    }


    // Tenant Info Validation and Saving
    std::string tenantResponse = validateAndSaveTenantInfo(doc);           // REPLACED String
    if (tenantResponse.find("saved successfully") == std::string::npos) {  // Use .find() and std::string::npos
      pref.end();
      writeResponse(400, 2, tenantResponse, "{}");
      return;
    }

    //Save Device config
    std::string deviceConfigResponse = validateAndSaveDeviceParameter(doc);      // REPLACED String
    if (deviceConfigResponse.find("saved successfully") == std::string::npos) {  // Use .find() and std::string::npos
      pref.end();
      writeResponse(400, 2, deviceConfigResponse, "{}");
      return;
    }

    doc.clear();
    // When ever configuration changes update toppics
    std::string teantId = pref.getString("tenant_id");  // Retrieve as std::string

    // Build the topics using std::string concatenation
    pref.putString("TOPIC_DATA", "int/iot/" + teantId + "/devices/" + deviceId + "/data");
    pref.putString("TOPIC_HEALTH", "int/iot/" + teantId + "/devices/" + deviceId + "/health");

    // End Preferences session
    pref.end();

    //Cleanup
    if (mqttClient.connected()) {
      mqttClient.disconnect();  // Disconnect from the MQTT broker
    }

    WiFi.disconnect();

    // If both Wi-Fi and MQTT credentials are saved successfully
    writeResponse(200, 2, "Device configuration saved successfully", "{}");
  }

  // Helper function to validate and save Tenant Info (tenant_id) (REPLACED String)
  std::string validateAndSaveTenantInfo(StaticJsonDocument<1024>& doc) {
    if (doc.containsKey("tenant_id")) {
      // Retrieve as C-string then convert to std::string
      std::string tenantId = doc["tenant_id"].as<const char*>();

      if (tenantId.empty()) {
        return "Tenant ID is empty or invalid.";
      }

      // Save Tenant ID to Preferences
      pref.putString("tenant_id", tenantId);
      return "Tenant ID saved successfully.";
    } else {
      return "Tenant ID missing in the configuration.";
    }
  }

  // Helper function to validate and save Tenant Info (tenant_id) (REPLACED String)
  std::string validateAndSaveDeviceParameter(StaticJsonDocument<1024>& doc) {

    // --- publish_interval ---
    if (doc.containsKey("publish_interval")) {
      int temp = doc["publish_interval"].as<int>();
      if (temp <= 0) {  // Changed == 0 to <= 0 for robustness
        return "publish_interval is invalid (must be positive).";
      }
      pref.putInt("publish_interval", temp);  // Note: using "publish_interval" key, not "p_interval" as in old code
    } else {
      return "publish_interval missing in the configuration.";
    }

    // --- scan_interval ---
    if (doc.containsKey("scan_interval")) {
      int scan_interval = doc["scan_interval"].as<int>();
      if (scan_interval <= 0) {
        return "scan_interval is invalid (must be positive).";
      }
      pref.putInt("scan_interval", scan_interval);
    } else {
      return "scan_interval missing in the configuration.";
    }

    // --- device_ttl ---
    if (doc.containsKey("device_ttl")) {
      int device_ttl = doc["device_ttl"].as<int>();
      if (device_ttl <= 0) {
        return "device_ttl is invalid (must be positive).";
      }
      pref.putInt("device_ttl", device_ttl);
    } else {
      return "device_ttl missing in the configuration.";
    }

    // --- rssi_filter ---
    if (doc.containsKey("rssi_filter")) {
      int rssi_filter = doc["rssi_filter"].as<int>();
      // Note: RSSI filter can be negative, so 0 check is enough if we assume it's set to a value
      if (rssi_filter == 0) {
        return "rssi_filter is invalid (cannot be 0).";
      }
      pref.putInt("rssi_filter", rssi_filter);
    } else {
      return "rssi_filter missing in the configuration.";
    }

    // --- geofence_radius ---
    if (doc.containsKey("geofence_radius")) {
      int geofence_radius = doc["geofence_radius"].as<int>();
      if (geofence_radius <= 0) {
        return "geofence_radius is invalid (must be positive).";
      }
      pref.putInt("geofence_radius", geofence_radius);
    } else {
      return "geofence_radius missing in the configuration.";
    }

    return "device parameters saved successfully.";
  }

  // Helper function to validate and save Wi-Fi credentials (REPLACED String)
  std::string validateWiFi(StaticJsonDocument<1024>& doc) {
    if (doc.containsKey("ssid") && doc.containsKey("ssid_password")) {
      // Retrieve as C-string then convert to std::string
      std::string ssid = doc["ssid"].as<const char*>();
      std::string ssidPassword = doc["ssid_password"].as<const char*>();

      if (ssid.empty() || ssidPassword.empty()) {
        return "Wi-Fi credentials missing or invalid.";
      }

      // Use .c_str() for WiFi.begin
      WiFi.begin(ssid.c_str(), ssidPassword.c_str());
      int timeout = 0;
      while (WiFi.status() != WL_CONNECTED && timeout < 20) {
        delay(500);
        timeout++;
      }

      if (WiFi.status() != WL_CONNECTED) {
        return "Failed to connect to Wi-Fi. Please check your credentials and network.";
      }

      // Save Wi-Fi credentials if valid
      pref.putString("ssid", ssid);
      pref.putEncryptedString("ssid_password", ssidPassword);
      return "Wi-Fi credentials saved successfully.";
    } else {
      return "Wi-Fi credentials missing in the configuration.";
    }
  }

  // Helper function to validate and save MQTT credentials (REPLACED String)
  std::string validateMQTT(StaticJsonDocument<1024>& doc) {
    if (doc.containsKey("mqtt_server") && doc.containsKey("mqtt_username") && doc.containsKey("mqtt_password") && doc.containsKey("mqtt_port")) {

      std::string mqttServer = doc["mqtt_server"].as<const char*>();      // REPLACED String
      std::string mqttUsername = doc["mqtt_username"].as<const char*>();  // REPLACED String
      std::string mqttPassword = doc["mqtt_password"].as<const char*>();  // REPLACED String
      int mqttPort = doc["mqtt_port"].as<int>();

      if (mqttServer.empty() || mqttPort <= 0) {
        return "MQTT server or port missing or invalid.";
      }

      // Use .c_str() for PubSubClient
      mqttClient.setServer(mqttServer.c_str(), mqttPort);
      // Use .c_str() for connect
      if (mqttClient.connect("ESP32Client", mqttUsername.c_str(), mqttPassword.c_str())) {
        // Save MQTT credentials if valid
        pref.putString("mqtt_server", mqttServer);
        pref.putString("mqtt_username", mqttUsername);
        pref.putEncryptedString("mqtt_password", mqttPassword);
        pref.putInt("mqtt_port", mqttPort);
        return "MQTT credentials saved successfully.";
      } else {
        return "Failed to connect to MQTT broker. Please check server address and credentials.";
      }
    } else {
      return "MQTT credentials missing in the configuration.";
    }
  }


  // Clear the saved configuration
  void clearConfiguration() {
    ///Serial.println("Clearing Configuration");
    WiFi.mode(WIFI_OFF);
    delay(3000);

    pref.begin("config", false);
    pref.clear();
    pref.end();
    writeResponse(200, 999, "Configuration cleared successfully", "{}");
  }

  // Retrieve the saved configuration (this command is forbidden)
  void retrieveConfiguration() {
    Serial.println("Retrieve Configuration");
    writeResponse(403, 3, "Forbidden", "{\"error\":\"Access to the configuration is forbidden.\"}");
  }

  // void deviceInfo(int topCount = 15) {
  //   StaticJsonDocument<2048> requestBody;

  //   // Format MAC address as 12-digit hex
  //   uint64_t mac = ESP.getEfuseMac();
  //   char macStr[13];
  //   snprintf(macStr, sizeof(macStr), "%012llx", mac);

  //   // Collect device information
  //   // Use .c_str() for assigning std::string to JsonDocument, or assign std::string directly
  //   requestBody["name"] = deviceName;
  //   requestBody["serial_number"] = macStr; // C-string is fine
  //   requestBody["model"] = ESP.getChipModel();
  //   requestBody["manufacturer"] = deviceManufacturer;
  //   requestBody["cpu_frequency_mhz"] = ESP.getCpuFreqMHz();
  //   requestBody["device_type"] = deviceType;
  //   requestBody["flash_size_mb"] = ESP.getFlashChipSize() / (1024 * 1024);
  //   requestBody["flash_speed_mhz"] = ESP.getFlashChipSpeed() / 1000000;

  //   pref.begin("config", false);
  //   // Retrieve as std::string
  //   requestBody["ssid"] = pref.getString("ssid");
  //   requestBody["ssid_password"] = pref.getDecryptedString("ssid_password");
  //   // Note: Corrected "p_interval" key to "publish_interval" based on logic in saveConfiguration
  //   requestBody["scan_interval"] = pref.getInt("scan_interval", 10000);
  //   requestBody["publish_interval"] = pref.getInt("publish_interval", 15000);
  //   requestBody["device_ttl"] = pref.getInt("device_ttl", 30000);
  //   requestBody["rssi_filter"] = pref.getInt("rssi_filter", -150);
  //   requestBody["geofence_radius"] = pref.getInt("geofence_radius", 8);
  //   pref.end();

  //   WiFi.mode(WIFI_STA);
  //   int numNetworks = WiFi.scanNetworks();

  //   std::vector<NetworkInfo> networks;

  //   for (int i = 0; i < numNetworks; i++) {
  //     // WiFi.SSID(i) returns an Arduino String, use .c_str() to convert to std::string
  //     networks.push_back({ WiFi.SSID(i).c_str(), WiFi.RSSI(i) });
  //   }

  //   std::sort(networks.begin(), networks.end(), [](const NetworkInfo& a, const NetworkInfo& b) {
  //     return a.rssi > b.rssi;
  //   });

  //   JsonArray ssidArray = requestBody.createNestedArray("available_ssids");
  //   for (int i = 0; i < std::min(topCount, (int)networks.size()); i++) {
  //     ssidArray.add(networks[i].ssid); // std::string is supported by ArduinoJson
  //   }

  //   WiFi.scanDelete();

  //   // Use std::string for the output stream
  //   std::string requestBodyStr;
  //   // Serialize to std::string
  //   serializeJson(requestBody, requestBodyStr);
  //   writeResponse(200, 0, "Success", requestBodyStr);
  // }

  void deviceInfo(int topCount = 15) {
    StaticJsonDocument<2048> requestBody;

    // Format MAC address as 12-digit hex
    uint64_t mac = ESP.getEfuseMac();
    char macStr[13];
    snprintf(macStr, sizeof(macStr), "%012llx", mac);

    // Collect device information
    requestBody["name"] = deviceName;
    requestBody["serial_number"] = macStr;
    requestBody["model"] = ESP.getChipModel();
    requestBody["manufacturer"] = deviceManufacturer;
    requestBody["cpu_frequency_mhz"] = ESP.getCpuFreqMHz();
    requestBody["device_type"] = deviceType;
    requestBody["flash_size_mb"] = ESP.getFlashChipSize() / (1024 * 1024);
    requestBody["flash_speed_mhz"] = ESP.getFlashChipSpeed() / 1000000;

    // Retrieve saved configuration parameters
    pref.begin("config", false);
    requestBody["ssid"] = pref.getString("ssid");
    // WARNING: Exposing password in 'deviceInfo' is a security risk for production
    requestBody["ssid_password"] = pref.getDecryptedString("ssid_password");
    requestBody["scan_interval"] = pref.getInt("scan_interval", 10000);
    requestBody["publish_interval"] = pref.getInt("publish_interval", 15000);
    requestBody["device_ttl"] = pref.getInt("device_ttl", 30000);
    requestBody["rssi_filter"] = pref.getInt("rssi_filter", -150);
    requestBody["geofence_radius"] = pref.getInt("geofence_radius", 8);
    pref.end();

    // --- Memory Optimized Wi-Fi Scan and Sort ---

    WiFi.mode(WIFI_STA);
    int numNetworks = WiFi.scanNetworks();

    // 1. Create a vector of indices [0, 1, 2, ...]
    // This avoids copying all SSIDs into memory.
    std::vector<int> indices(numNetworks);
    for (int i = 0; i < numNetworks; ++i) {
      indices[i] = i;
    }

    // 2. Sort the indices based on the RSSI of the corresponding network
    // Using a lambda to access WiFi.RSSI(index) for comparison.
    std::sort(indices.begin(), indices.end(), [&](int i, int j) {
      return WiFi.RSSI(i) > WiFi.RSSI(j);  // Sort descending (strongest RSSI first)
    });

    JsonArray ssidArray = requestBody.createNestedArray("available_ssids");

    // 3. Iterate over the sorted indices and add the top SSIDs to the JSON array
    for (int i = 0; i < std::min(topCount, numNetworks); i++) {
      int sortedIndex = indices[i];
      // Access the SSID using the sorted index and convert Arduino String to C-string
      ssidArray.add(WiFi.SSID(sortedIndex).c_str());
    }

    WiFi.scanDelete();

    // --- End Wi-Fi Scan and Sort ---

    // Use std::string for the output stream
    std::string requestBodyStr;
    // Serialize to std::string
    serializeJson(requestBody, requestBodyStr);
    writeResponse(200, 0, "Success", requestBodyStr);
  }

  // REPLACED String with const std::string&
  void writeResponse(int status_code, int request_code, const std::string& status_message, const std::string& payload) {
    std::string json;
    json.reserve(2048);  // Reserve memory for efficiency

    // Use std::to_string for integer-to-string conversion
    json += "{\"status_code\":";
    json += std::to_string(status_code);
    json += ",\"request_code\":";
    json += std::to_string(request_code);
    json += ",\"status_message\":\"";
    json += status_message;
    json += "\",\"payload\":";
    json += payload;
    json += "}";
    //json += "\n";

    Serial.println(json.c_str());  // Use .c_str() for Serial.println
  }
};
#endif  // DEVICEMANAGER_H    `