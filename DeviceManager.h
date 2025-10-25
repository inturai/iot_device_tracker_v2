/*
 ^ The Inturai License
 * 
 * Copyright (c) 2024 Inturai. All Rights Reserved.
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
#include <SecureStorage.h>
//#include<Preferences.h>
#include <WiFiClientSecure.h>

// Define the maximum buffer sizes for various configurations
#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64
#define MAX_MQTT_SERVER_LENGTH 128
#define MAX_MQTT_USERNAME_LENGTH 64
#define MAX_MQTT_PASSWORD_LENGTH 64
#define MAX_MQTT_PORT_LENGTH 5

  class DeviceManager {
private:
  SecureStorage pref;  // For saving configurations in ESP32 Preferences
  WiFiClient espClient;
  //WiFiClientSecure espClient;

  PubSubClient mqttClient;  // MQTT client

  String deviceName;          // Device name
  int deviceType;             // Device type
  String deviceManufacturer;  // Device manufacturer
  String deviceVersion;       // Device version

  //Device id and Tenant ID
  String teanantId;

  const char* ca_cert;
public:
  // Constructor for initialization with name, type, manufacturer, and version of the device
  DeviceManager(String name, int type, String manufacturer, String version, const char* certificate)
    : mqttClient(espClient), deviceName(name), deviceType(type), deviceManufacturer(manufacturer), deviceVersion(version), ca_cert(certificate) {
    // Constructor initializes deviceName, deviceType, deviceManufacturer, and deviceVersion
    // Other initialization tasks can go here
    //espClient.setCACert(ca_cert);  // Set the CA certificate
  }

  //Returns Tenant ID
  String getTenantId() {
    return teanantId;
  }

  // Method to handle incoming commands
  int handleCommand(String commandString) {
    commandString.trim();

    // Ensure command follows the 'command:payload' format
    int colonIndex = commandString.indexOf(':');
    if (colonIndex == -1) {
      writeResponse(400, -1, "Invalid Command Format", "{\"error\":\"Command must follow the format 'command:payload'\"}");
      return -1;
    }

    int commandCode = commandString.substring(0, colonIndex).toInt();
    String payload = commandString.substring(colonIndex + 1);

    // Serial.print("Received payload: ");
    // Serial.println(payload);

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

  // Save the configuration from the payload
  void saveConfiguration(String payload) {

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);

    // Check if JSON deserialization was successful
    if (error) {
      writeResponse(400, 2, "Invalid JSON format", "{\"error\":\"Invalid JSON format\"}");
      return;
    }

    // Initialize Preferences
    pref.begin("config", false);

    String deviceId;
    if (doc.containsKey("device_id")) {
      deviceId = doc["device_id"].as<String>();
      if (!deviceId.isEmpty()) {
        pref.putString("device_id", deviceId);
      } else {
        deviceId = String(ESP.getEfuseMac(), HEX);
      }
    } else {
      deviceId = String(ESP.getEfuseMac(), HEX);
    }

    //Set device_id also into preferences
    //String deviceId = String(ESP.getEfuseMac(), HEX);
    //pref.putString("device_id", deviceId);

    String statusMessage = "";

    // Wi-Fi Credentials Validation
    String wifiResponse = validateWiFi(doc);
    if (wifiResponse.indexOf("saved successfully") == -1) {
      // If Wi-Fi fails, return the response
      pref.end();
      writeResponse(400, 2, wifiResponse, "{}");
      return;
    }

    // MQTT Credentials Validation (Only after Wi-Fi success)
    String mqttResponse = validateMQTT(doc);
    if (mqttResponse.indexOf("saved successfully") == -1) {
      // If MQTT fails, return the response
      pref.end();
      writeResponse(400, 2, mqttResponse, "{}");
      return;
    }


    // Tenant Info Validation and Saving
    String tenantResponse = validateAndSaveTenantInfo(doc);
    if (tenantResponse.indexOf("saved successfully") == -1) {
      // If Tenant ID validation fails, return the response
      pref.end();
      writeResponse(400, 2, tenantResponse, "{}");
      return;
    }


    //When ever configuration changes update toppics
    String teantId = pref.getString("tenant_id");

    pref.putString("TOPIC_DATA", "int/loc/" + teantId + "/devices/" + deviceId + "/data");
    pref.putString("TOPIC_STATUS", "int/loc/" + teantId + "/devices/" + deviceId + "/status");
    pref.putString("TOPIC_EVENT", "int/loc/" + teantId + "/devices/" + deviceId + "/event");
    //By the this time, we will have tenant_id in preferences
    pref.putString("TOPIC_HEALTH", "int/loc/" + teantId + "/devices/" + deviceId + "/health");


    // Device specific properties
    // RSSI Info Validation and Saving
    String rssiCalibrationResponse = validateAndSaveRSSICalibration(doc);
    if (rssiCalibrationResponse.indexOf("saved successfully") == -1) {
      // If Tenant ID validation fails, return the response
      pref.end();
      writeResponse(400, 2, rssiCalibrationResponse, "{}");
      return;
    }

    String rssiFilterResponse = validateAndSaveRSSIFilter(doc);
    if (rssiFilterResponse.indexOf("saved successfully") == -1) {
      // If Tenant ID validation fails, return the response
      pref.end();
      writeResponse(400, 2, rssiFilterResponse, "{}");
      return;
    }

    // End Preferences session
    pref.end();

    //Cleanup
    if (mqttClient.connected()) {
      mqttClient.disconnect();  // Disconnect from the MQTT broker
      //Serial.println("Disconnected from MQTT broker.");
    }

    WiFi.disconnect();
    //Serial.println("Disconnected from WiFi.");

    // If both Wi-Fi and MQTT credentials are saved successfully
    writeResponse(200, 2, "Device configuration saved successfully", "{}");
  }

  // Helper function to validate and save Tenant Info (tenant_id)
  String validateAndSaveTenantInfo(StaticJsonDocument<512>& doc) {
    if (doc.containsKey("tenant_id")) {
      String tenantId = doc["tenant_id"].as<String>();

      if (tenantId.isEmpty()) {
        return "Tenant ID is empty or invalid.";
      }

      // Optionally, you can add more validation rules here for tenant_id format

      // Save Tenant ID to Preferences
      pref.putString("tenant_id", tenantId);
      return "Tenant ID saved successfully.";
    } else {
      return "Tenant ID missing in the configuration.";
    }
  }


  // Helper function to validate and save RSSI nfo rssi_calibration
  String validateAndSaveRSSICalibration(StaticJsonDocument<512>& doc) {

    //rssi_calibration
    if (doc.containsKey("rssi_calibration")) {
      int rssiCalibration = doc["rssi_calibration"].as<int>();

      if (rssiCalibration == 0) {
        return "rssi_calibration is invalid.";
      }
      // Save Tenant ID to Preferences
      pref.putInt("rssi_cali", rssiCalibration);
      return "rssi_calibration saved successfully.";
    } else {
      return "rssi_calibration missing in the configuration.";
    }
  }



  // Helper function to validate and save RSSI nfo rssi_filter
  String validateAndSaveRSSIFilter(StaticJsonDocument<512>& doc) {
    //rssi_filter
    if (doc.containsKey("rssi_filter")) {
      int rssiFilter = doc["rssi_filter"].as<int>();

      if (rssiFilter == 0) {
        return "rssi_filter or invalid.";
      }
      // Save Tenant ID to Preferences
      pref.putInt("rssi_filter", rssiFilter);
      return "rssi_filter saved successfully.";
    } else {
      return "rssi_filter missing in the configuration.";
    }
  }

  // Helper function to validate and save Wi-Fi credentials
  String validateWiFi(StaticJsonDocument<512>& doc) {
    if (doc.containsKey("ssid") && doc.containsKey("ssid_password")) {
      String ssid = doc["ssid"].as<String>();
      String ssidPassword = doc["ssid_password"].as<String>();

      if (ssid.isEmpty() || ssidPassword.isEmpty()) {
        return "Wi-Fi credentials missing or invalid.";
      }

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

  // Helper function to validate and save MQTT credentials
  String validateMQTT(StaticJsonDocument<512>& doc) {
    if (doc.containsKey("mqtt_server") && doc.containsKey("mqtt_username") && doc.containsKey("mqtt_password") && doc.containsKey("mqtt_port")) {

      String mqttServer = doc["mqtt_server"].as<String>();
      String mqttUsername = doc["mqtt_username"].as<String>();
      String mqttPassword = doc["mqtt_password"].as<String>();
      int mqttPort = doc["mqtt_port"].as<int>();

      if (mqttServer.isEmpty() || mqttPort <= 0) {
        return "MQTT server or port missing or invalid.";
      }

      mqttClient.setServer(mqttServer.c_str(), mqttPort);
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
    WiFi.mode(WIFI_OFF);  // otherwise Wifi.Scannetworks() fails
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

  // // Provide device information (like chip details, but no Wi-Fi info now)
  // void deviceInfo() {
  //   StaticJsonDocument<1024> requestBody;  // Increased the size of the document to accommodate SSIDs

  //   // Collect device information
  //   requestBody["name"] = deviceName;
  //   requestBody["serial_number"] = String(ESP.getEfuseMac(), HEX);
  //   requestBody["model"] = ESP.getChipModel();
  //   requestBody["manufacturer"] = deviceManufacturer;
  //   requestBody["cpu_frequency"] = String(ESP.getCpuFreqMHz()) + " MHz";
  //   requestBody["device_type"] = deviceType;
  //   requestBody["flash_size"] = String(ESP.getFlashChipSize() / (1024 * 1024)) + " MB";
  //   requestBody["flash_speed"] = String(ESP.getFlashChipSpeed() / 1000000) + " MHz";
  //   pref.begin("config", false);
  //   requestBody["ssid"] = pref.getString("ssid");
  //   requestBody["ssid_password"] = pref.getString("ssid_password", "");
  //   pref.end();

  //   WiFi.mode(WIFI_OFF); // otherwise Wifi.Scannetworks() fails

  //   // Scan for available Wi-Fi networks
  //   int numNetworks = WiFi.scanNetworks();
  //   JsonArray ssidArray = requestBody.createNestedArray("available_ssids");

  //   // Loop through the networks and add SSIDs to the array
  //   for (int i = 0; i < numNetworks; i++) {
  //     ssidArray.add(WiFi.SSID(i));  // Add SSID to the JSON array

  //     // JsonObject network = ssidArray.createNestedObject();
  //     // network["name"] = WiFi.SSID(i);  // Store the SSID name
  //     // network["rssi"] = WiFi.RSSI(i);  // Store the RSSI value
  //   }

  //   // Serialize the JSON object to a string
  //   String requestBodyStr;
  //   serializeJson(requestBody, requestBodyStr);

  //   WiFi.scanDelete(); // Free memory

  //   writeResponse(200, 0, "Success", requestBodyStr);
  // }

  void deviceInfo() {
    StaticJsonDocument<1024> requestBody;

    // Collect device information
    requestBody["name"] = deviceName;
    requestBody["serial_number"] = String(ESP.getEfuseMac(), HEX);
    requestBody["model"] = ESP.getChipModel();
    requestBody["manufacturer"] = deviceManufacturer;
    requestBody["cpu_frequency"] = String(ESP.getCpuFreqMHz()) + " MHz";
    requestBody["device_type"] = deviceType;
    requestBody["flash_size"] = String(ESP.getFlashChipSize() / (1024 * 1024)) + " MB";
    requestBody["flash_speed"] = String(ESP.getFlashChipSpeed() / 1000000) + " MHz";

    pref.begin("config", false);
    requestBody["ssid"] = pref.getString("ssid");
    requestBody["ssid_password"] = pref.getDecryptedString("ssid_password");
    pref.end();

    // Ensure Wi-Fi is off before scanning
    WiFi.mode(WIFI_OFF);
    delay(100);           // Give hardware time to reset WiFi
    WiFi.mode(WIFI_STA);  // Must be in station mode for scanning
    delay(100);

    int numNetworks = WiFi.scanNetworks();
    const int MAX_SSIDS = 15;

    // Struct to hold index and RSSI
    struct NetworkInfo {
      int index;
      int rssi;
    };

    NetworkInfo topNetworks[MAX_SSIDS];
    int count = 0;

    for (int i = 0; i < numNetworks; i++) {
      int rssi = WiFi.RSSI(i);
      int insertPos = -1;

      for (int j = 0; j < count; j++) {
        if (rssi > topNetworks[j].rssi) {
          insertPos = j;
          break;
        }
      }

      if (insertPos == -1 && count < MAX_SSIDS) {
        topNetworks[count++] = { i, rssi };
      } else if (insertPos != -1) {
        if (count < MAX_SSIDS) count++;
        for (int j = count - 1; j > insertPos; j--) {
          topNetworks[j] = topNetworks[j - 1];
        }
        topNetworks[insertPos] = { i, rssi };
      }
    }

    // Create JSON array of top SSIDs
    JsonArray ssidArray = requestBody.createNestedArray("available_ssids");
    for (int i = 0; i < count; i++) {
      String ssid = WiFi.SSID(topNetworks[i].index);
      if (ssid.length() > 0) {
        ssidArray.add(ssid);
      }
    }

    WiFi.scanDelete();  // Free memory after scan

    // Serialize to string
    String requestBodyStr;
    serializeJson(requestBody, requestBodyStr);

    writeResponse(200, 0, "Success", requestBodyStr);
  }


  // Method to write the response in JSON format
  void writeResponse(int status_code, int request_code, String status_message, String payload) {
    Serial.print("{\"status_code\":");
    Serial.print(status_code);
    Serial.print(",\"request_code\":");
    Serial.print(request_code);
    Serial.print(",\"status_message\":\"");
    Serial.print(status_message);
    Serial.print("\",\"payload\":");
    Serial.print(payload);
    Serial.print("}\n");
  }
};
#endif  // DEVICEMANAGER_H