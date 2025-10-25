#include <WiFi.h>
#include <SecureStorage.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include "DeviceManager.h"
#include <WiFiClientSecure.h>


#include <BLEDevice.h>  // For BLE functionality

#define SERIAL_BUFFER_SIZE 2048  // Increase buffer size to 2048 bytes
#define MQTT_BUFFER_SIZE 2048

// Serial configuration
#define BAUD_RATE 115200

// SSL Certificate (you can replace this with your CA certificate string)
const char* ca_cert = R"rawliteral(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)rawliteral";

// Create a global instance of DeviceManager
DeviceManager deviceManager("Triangulation Access Point", 4, "Seeed Studio", "1.0.9", ca_cert);

// MQTT client and configuration
WiFiClient espClient;
//WiFiClientSecure espClient;

PubSubClient mqttClient(espClient);

// Configuration info for WiFi and MQTT
String tenant_id = "";
String device_id = "";
String savedSSID = "";
String savedPassword = "";
String mqttServer = "";
int mqttPort = 1883;
String mqttUsername = "";
String mqttPassword = "";

// MQTT topics
String TOPIC_DATA = "";
String TOPIC_EVENT = "";
String TOPIC_HEALTH = "";
String TOPIC_STATUS = "";

//Device specific parameters
BLEScan* pBLEScan;

int rssi_calibration = -55;
int rssi_filter = -100;

// Create a new HardwareSerial class (for custom serial communication if needed)
HardwareSerial MySerial(1);

// JSON document for outgoing message
// StaticJsonDocument<2048> doc;
// char data[2048];

DynamicJsonDocument doc(2048);  // JSON doc allocated on the heap

// Preferences for saving data (e.g., WiFi and MQTT configurations)
SecureStorage preferences;

// Connectivity status
bool isConfigured = false;
unsigned long lastSentTime = 0;
const unsigned long HEALTH_PUBLISH_INTERVAL = 30000;  // 30 seconds

// Utility functions

/**
 * Helper function to create a JSON payload for event data.
 * @param doc The JSON document to which the data will be added.
 * @param eventType Type of event (e.g., "warning", "alert").
 * @param description Description of the event.
 * @param severity Severity level (e.g., "high", "low").
 * @param actualValue Optional actual value associated with the event.
 * @param alertMessage Optional alert message related to the event.
 * @return The created JSON object.
 */
JsonObject createJsonData(JsonDocument& doc, String eventType, String description, String severity, String actualValue = "", String alertMessage = "") {
  JsonObject eventNode = doc.createNestedObject("e");
  eventNode["tenant_id"] = tenant_id;
  eventNode["serial_number"] = device_id;
  eventNode["event_type"] = eventType;
  eventNode["description"] = description;
  eventNode["severity"] = severity;

  // Add optional fields if provided
  if (!actualValue.isEmpty()) {
    eventNode["actual_value"] = actualValue;
  }
  if (!alertMessage.isEmpty()) {
    eventNode["alert_message"] = alertMessage;
  }

  return eventNode;
}

/**
 * Connect to WiFi using stored credentials.
 * @param ssid SSID of the WiFi network.
 * @param password Password for the WiFi network.
 * @param tenant_id Tenant identifier for the device.
 * @return True if connected to WiFi, false if connection fails.
 */
bool connectToWiFi(const char* ssid, const char* password, const char* tenant_id) {
  // Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  int attempt = 0;
  const int maxAttempts = 5;
  const int baseDelay = 1000;  // 3 seconds

  while (WiFi.status() != WL_CONNECTED && attempt < maxAttempts && !Serial.available() > 0) {
    int delayTime = baseDelay * (1 << attempt);  // 2^attempt * baseDelay
    // Serial.printf("\nAttempt %d failed. Retrying in %d seconds...\n", attempt + 1, delayTime / 1000);
    delay(delayTime);
    attempt++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
    return true;
  } else {
    Serial.println("Failed to connect to WiFi.");
    return false;
  }
}

/**
 * Connect to MQTT broker with retry logic.
 * @param mqttServer MQTT broker address.
 * @param mqttPort MQTT broker port.
 * @param mqttUsername Username for MQTT login.
 * @param mqttPassword Password for MQTT login.
 * @return True if successfully connected to MQTT broker, false if connection fails.
 */
bool connectToMqtt(const char* mqttServer, int mqttPort, const char* mqttUsername, const char* mqttPassword) {
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);  // Optional

  int attempt = 0;
  const int maxAttempts = 5;
  const int baseDelay = 1000;  // 1 seconds

  while (!mqttClient.connected() && attempt < maxAttempts && !Serial.available() > 0) {
    // Serial.printf("Connecting to MQTT... Attempt %d\n", attempt + 1);

    if (mqttClient.connect(device_id.c_str(), mqttUsername, mqttPassword)) {
      //Serial.println("Connected to MQTT broker");
      return true;
    } else {
      //Serial.print("MQTT connection failed, rc=");
      //Serial.print(mqttClient.state());

      int delayTime = baseDelay * (1 << attempt);  // Exponential backoff: 2^attempt * baseDelay
      //Serial.printf(". Retrying in %d seconds...\n", delayTime / 1000);
      attempt++;
      delay(delayTime);
    }
  }

  // Serial.println("Failed to connect to MQTT after multiple attempts.");
  return false;
}


/**
 * Generate a JSON payload with system health data (e.g., uptime).
 * @return JSON-formatted health data as a string.
 */
String getHealthData() {
  unsigned long uptime = millis() / 1000;  // Get uptime in seconds
  String payload = "{";
  payload += "\"status\":\"active\",";
  payload += "\"uptime_seconds\": " + String(uptime);
  payload += "}";

  return payload;
}

/**
 * Helper function to publish data to MQTT broker. Ensures WiFi and MQTT connectivity.
 * @param topic MQTT topic to publish to.
 * @param payload Data to be sent in the message.
 * @return True if data was published successfully, false if failed.
 */
bool publishData(const String& topic, const String& payload) {
  // Ensure WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    //Serial.println("WiFi disconnected. Attempting to reconnect...");
    if (!connectToWiFi(savedSSID.c_str(), savedPassword.c_str(), tenant_id.c_str())) {
      //Serial.println("Failed to reconnect to WiFi.");
      return false;
    }
  }

  // Ensure MQTT is connected
  if (!mqttClient.connected()) {
    //Serial.println("MQTT disconnected. Attempting to reconnect...");
    if (!connectToMqtt(mqttServer.c_str(), mqttPort, mqttUsername.c_str(), mqttPassword.c_str())) {
      //Serial.println("Failed to reconnect to MQTT broker.");
      return false;
    }
  }

  // Publish data to the specified MQTT topic
  if (mqttClient.publish(topic.c_str(), payload.c_str())) {
    return true;
  } else {
    Serial.println("Failed to publish data.");
    return false;
  }
}

/**
 * Load the device configuration from stored preferences.
 * @return True if configuration is successfully loaded, false if any required settings are missing.
 */
bool loadConfig() {
  preferences.begin("config", false);  // Open the preferences with 'config' namespace

  device_id = preferences.getString("device_id", "");
  if (device_id.isEmpty()) {
    device_id = String(ESP.getEfuseMac(), HEX);  // Generate device ID based on MAC address
    //device_id.toUpperCase();
    preferences.putString("device_id", device_id);  // Store the generated device ID
  }

  // Load Wi-Fi and MQTT settings from preferences
  savedSSID = preferences.getString("ssid", "");
  savedPassword = preferences.getDecryptedString("ssid_password");
  tenant_id = preferences.getString("tenant_id", "");
  mqttServer = preferences.getString("mqtt_server", "");
  mqttPort = preferences.getInt("mqtt_port", 1883);
  mqttUsername = preferences.getString("mqtt_username", "");
  mqttPassword = preferences.getDecryptedString("mqtt_password");

  TOPIC_DATA = preferences.getString("TOPIC_DATA");
  TOPIC_EVENT = preferences.getString("TOPIC_EVENT");
  TOPIC_HEALTH = preferences.getString("TOPIC_HEALTH");
  TOPIC_STATUS = preferences.getString("TOPIC_STATUS");

  //Device specific configuration
  rssi_calibration = preferences.getInt("rssi_cali", -55);
  rssi_filter = preferences.getInt("rssi_filter", -100);

  preferences.end();

  // Validate that all required settings are loaded
  if (device_id.isEmpty() || savedSSID.isEmpty() || savedPassword.isEmpty() || tenant_id.isEmpty()
      || mqttServer.isEmpty() || TOPIC_DATA.isEmpty() || TOPIC_EVENT.isEmpty() || TOPIC_HEALTH.isEmpty()
      || TOPIC_STATUS.isEmpty() || rssi_calibration == 0 || rssi_filter == 0) {
    // Serial.println("Configuration missing or invalid.");
    return false;
  }

  return true;
}

/**
 * Clear all stored configuration settings.
 */
void clearConfig() {
  device_id = "";
  savedSSID = "";
  savedPassword = "";
  mqttServer = "";
  mqttPort = 1883;
  mqttUsername = "";
  mqttPassword = "";

  TOPIC_DATA = "";
  TOPIC_EVENT = "";
  TOPIC_HEALTH = "";
  TOPIC_STATUS = "";
  rssi_filter = -100;
  rssi_calibration = -55;
  isConfigured = false;
}

// Utility functions

// Structure to hold Wi-Fi network information
struct WiFiDevice {
  String SSID;
  int RSSI;
};

// Structure to hold BLE device information
struct BLEDeviceInfo {
  String Name;
  String Address;
  int RSSI;
};

#define NO_OF_WIFI_RECORDS 50
#define NO_OF_BLE_RECORDS 50

void scanDevicesAndPublish() {
  DynamicJsonDocument doc(2048);  // Use heap instead of stack
  JsonArray wifiArray = doc.createNestedArray("wifi");
  JsonArray bleArray = doc.createNestedArray("ble");

  std::vector<WiFiDevice> wifiDevices;

  int numWiFiNetworks = WiFi.scanNetworks(false, true);
  if (numWiFiNetworks < 0) {
    // Serial.println("Wi-Fi scan failed.");
    return;
  }

  for (int i = 0; i < numWiFiNetworks; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.isEmpty()) {
      continue;
    }
    int rssi = WiFi.RSSI(i);
    if (rssi >= rssi_filter) {
      wifiDevices.push_back({ ssid, rssi });
    }
  }

  std::sort(wifiDevices.begin(), wifiDevices.end(), [](const WiFiDevice& a, const WiFiDevice& b) {
    return a.RSSI > b.RSSI;
  });

  for (int i = 0; i < std::min(NO_OF_WIFI_RECORDS, (int)wifiDevices.size()); i++) {
    JsonObject wifiDevice = wifiArray.createNestedObject();
    wifiDevice["SSID"] = wifiDevices[i].SSID;
    wifiDevice["RSSI"] = wifiDevices[i].RSSI;
  }

  std::vector<BLEDeviceInfo> bleDevices;
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(200);
  pBLEScan->setWindow(150);
  BLEScanResults* foundDevices = pBLEScan->start(3, true);

  for (int i = 0; i < foundDevices->getCount(); i++) {
    BLEAdvertisedDevice advertisedDevice = foundDevices->getDevice(i);
    String name = advertisedDevice.getName();
    String address = advertisedDevice.getAddress().toString().c_str();
    int rssi = advertisedDevice.getRSSI();

    // if (name.isEmpty()) name = "Unknown";
    //IF name not empty
    if (name.isEmpty())
      if (name.isEmpty()) {
        continue;
      }
    {
      name = parseNameFromPayload(advertisedDevice.getPayload(), advertisedDevice.getPayloadLength());
      if (name == "(no name)") {
        String manufacturer = getManufacturerName(advertisedDevice);
        //name = manufacturer + " [" + address + "]";
        name = address + " [" + manufacturer + "]";
      }
    }

    bleDevices.push_back({ name, address, rssi });
  }

  std::sort(bleDevices.begin(), bleDevices.end(), [](const BLEDeviceInfo& a, const BLEDeviceInfo& b) {
    return a.RSSI > b.RSSI;
  });

  for (int i = 0; i < std::min(NO_OF_BLE_RECORDS, (int)bleDevices.size()); i++) {
    JsonObject bleDevice = bleArray.createNestedObject();
    bleDevice["Name"] = bleDevices[i].Name;
    bleDevice["Address"] = bleDevices[i].Address;
    bleDevice["RSSI"] = bleDevices[i].RSSI;
  }

  doc["device_id"] = device_id;

  String data;
  serializeJson(doc, data);
  if (!publishData(TOPIC_DATA, data)) {
    Serial.println("Failed to publish WiFi and BLE data.");
  }

  WiFi.scanDelete();
  if (pBLEScan != NULL) pBLEScan->clearResults();
  wifiDevices.clear();
  bleDevices.clear();
}


String parseNameFromPayload(const uint8_t* payload, size_t length) {
  size_t index = 0;

  while (index < length) {
    uint8_t fieldLen = payload[index];
    if (fieldLen == 0 || (index + fieldLen + 1 > length)) break;

    uint8_t fieldType = payload[index + 1];

    if (fieldType == 0x09 || fieldType == 0x08) {
      return String((const char*)(payload + index + 2), fieldLen - 1);
    }

    index += fieldLen + 1;
  }

  return "(no name)";
}

String getManufacturerName(BLEAdvertisedDevice& device) {
  if (!device.haveManufacturerData()) return "UNKN";

  String rawData = device.getManufacturerData();
  const uint8_t* data = (const uint8_t*)rawData.c_str();
  size_t len = rawData.length();

  if (len < 2) return "UNKN";

  uint16_t companyId = data[0] | (data[1] << 8);

  switch (companyId) {
    case 0x004C: return "APPL";  // Apple Inc.
    case 0x0006: return "MSFT";  // Microsoft Corporation
    case 0x000F: return "BRDC";  // Broadcom Corporation
    case 0x0009: return "MOTO";  // Motorola
    case 0x000A: return "TOSH";  // Toshiba Corporation
    case 0x000C: return "QLCM";  // Qualcomm Technologies
    case 0x003D: return "SMSG";  // Samsung Electronics Co.
    case 0x0059: return "SONY";  // Sony Corporation
    case 0x00E0: return "FITB";  // Fitbit Inc.
    case 0x0131: return "XIAO";  // Xiaomi Inc.
    case 0x0133: return "HUAW";  // Huawei Technologies Co.
    case 0x0171: return "GOOG";  // Google LLC
    case 0x01DA: return "BOSE";  // Bose Corporation
    case 0x01E3: return "GRMN";  // Garmin International
    case 0x01F4: return "GPRO";  // GoPro Inc.
    case 0x0211: return "TILE";  // Tile Inc.
    case 0x022D: return "RLTK";  // Realtek Semiconductor
    case 0x027D: return "ONEP";  // OnePlus Technology
    case 0x02E0: return "NOTH";  // Nothing Technology Ltd.
    default: return "UNKN";      // Unknown Manufacturer
  }
}

// Setup and main loop

/**
 * Initialize system settings and connections at startup.
 */
void setup() {
  delay(1000);
  Serial.begin(BAUD_RATE);  // Initialize serial communication
  //MySerial.begin(BAUD_RATE, SERIAL_8N1, 4, 5);  // Initialize custom serial port (if needed)
  Serial.setRxBufferSize(2048);  // Set buffer size to 2048 bytes

  // Set the MQTT client options (use SSL/TLS certificate)
  //espClient.setCACert(ca_cert);  // Set the CA certificate

  // //Warning following while loop causes infite loop when connected via battery
  // while (!Serial)
  //   ;  // Wait for serial port to initialize (required for some boards)
  if (Serial)
    while (!Serial)
      ;

  isConfigured = loadConfig();  // Load configuration

  BLEDevice::init("");

  if (isConfigured) {
    publishData(TOPIC_STATUS.c_str(), "{}");  // Publish initial status to MQTT
  }
}

/**
 * Main loop for checking system health and handling commands.
 */
void loop() {
  // Handle serial commands (e.g., reset config)
  if (Serial.available() > 0) {
    String commandString = Serial.readStringUntil('\n');
    int commandId = deviceManager.handleCommand(commandString);  // Handle received command
    switch (commandId) {
      case 999:
        clearConfig();
        break;  // Clear all configuration if command ID 999 is received
      case 2:
        isConfigured = loadConfig();
        break;  // Reload configuration
    }
    //delay(100);  // Prevent overloading the processor
  } else if (isConfigured && !Serial.available()) {

    scanDevicesAndPublish();
    //Health monitor
    unsigned long currentMillis = millis();
    if (currentMillis - lastSentTime >= HEALTH_PUBLISH_INTERVAL) {
      String healthData = getHealthData();  // Get health data
      if (!publishData(TOPIC_HEALTH.c_str(), healthData.c_str())) {
        Serial.println("Failed to publish health data");
      }
      lastSentTime = currentMillis;  // Update last sent time
    }
  }

  delay(1000);
}