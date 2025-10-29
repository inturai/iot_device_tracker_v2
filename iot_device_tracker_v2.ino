#include <string>   // Include the standard string library
#include <sstream>  // For number to string conversion (std::to_string or stringstream)

//#include <Preferences.h>
#include "DeviceScanner.h"
#include "DeviceManager.h"
#include <SecureStorage.h>
#include <PubSubClient.h>  // Keep this here to ensure PubSubClient is available
#include "country.h"

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
  TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u/uNxe5AW0wdeRlN8NwdC
  jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb/ZAJzVc
  oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
  4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
  mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
  emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
  -----END CERTIFICATE-----
  )rawliteral";

#define SERIAL_BUFFER_SIZE 1024

// Configuration info for WiFi and MQTT (REPLACED String with std::string)
std::string tenant_id = "";
std::string device_id = "";
std::string savedSSID = "";
std::string savedPassword = "";
std::string mqttServer = "";
int mqttPort = 1883;
std::string mqttUsername = "";
std::string mqttPassword = "";

// MQTT topics (REPLACED String with std::string)
std::string TOPIC_DATA = "";
std::string TOPIC_HEALTH = "";

// Device specific parameters
int rssi_filter = -100;
int geofence_radius = 8;

int scan_interval = 10000;

int publish_interval = 15000;
int device_ttl = 10000;
std::string country = "AU";

bool isConfigured = false;
unsigned long lastSentTime = 0;
const unsigned long HEALTH_PUBLISH_INTERVAL = 30000;  // 30 seconds

// Preferences for saving data (e.g., WiFi and MQTT configurations)
SecureStorage preferences;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

DeviceScannerV4* deviceScannerV4 = nullptr;
DeviceManager deviceManager("Activity Access Point V2", 4, "Seeed Studio", "2.0", ca_cert);

/**
  * Load the device configuration from stored preferences.
  * @return True if configuration is successfully loaded, false if any required settings are missing.
  */
bool loadConfig() {
  preferences.begin("config", false);  // Open the preferences with 'config' namespace

  // GetString now returns Arduino String, so we use its .c_str() to convert to std::string
  device_id = preferences.getString("device_id", "").c_str();
  if (device_id.empty()) {  // Use .empty() instead of .isEmpty()
    // Generate device ID based on MAC address
    std::ostringstream mac_stream;
    mac_stream << std::hex << ESP.getEfuseMac();
    device_id = mac_stream.str();
    // Use .c_str() for storing back to preferences (which expects C-string or Arduino String)
    preferences.putString("device_id", device_id.c_str());
  }

  // Load Wi-Fi and MQTT settings from preferences
  savedSSID = preferences.getString("ssid", "").c_str();
  savedPassword = preferences.getDecryptedString("ssid_password").c_str();
  tenant_id = preferences.getString("tenant_id", "").c_str();
  mqttServer = preferences.getString("mqtt_server", "").c_str();
  mqttPort = preferences.getInt("mqtt_port", 1883);
  mqttUsername = preferences.getString("mqtt_username", "").c_str();
  mqttPassword = preferences.getDecryptedString("mqtt_password").c_str();

  // "scan_interval":10000,"publish_interval":15000,"device_ttl":30000
  scan_interval = preferences.getInt("scan_interval", 10000);
  publish_interval = preferences.getInt("pub_int", 15000);
  device_ttl = preferences.getInt("device_ttl", 10000);

  TOPIC_DATA = preferences.getString("TOPIC_DATA", "").c_str();
  TOPIC_HEALTH = preferences.getString("TOPIC_HEALTH", "").c_str();

  //Device specific configuration
  rssi_filter = preferences.getInt("rssi_filter", -100);
  geofence_radius = preferences.getInt("geofence_radius", 8);
  country = preferences.getString("country", "AU").c_str();

  preferences.end();

  // Validate that all required settings are loaded. Use .empty()
  if (device_id.empty() || savedSSID.empty() || savedPassword.empty() || tenant_id.empty()
      || mqttServer.empty() || TOPIC_DATA.empty() || TOPIC_HEALTH.empty()
      // Check for zero/invalid values
      || rssi_filter == 0 || scan_interval == 0 || publish_interval == 0 || device_ttl == 0 || geofence_radius == 0) {
    // Serial.println("Configuration missing or invalid.");
    return false;
  } else {
    if (deviceScannerV4) {
      delete deviceScannerV4;
    }

    // Pass std::string objects directly to the constructor (which now accepts std::string)
    // --- CRITICAL FIX: Removed the 18th argument (3000) and added casts ---
    // The constructor now expects 17 arguments.
    // Pass std::string objects directly to the constructor (which now accepts std::string)
    deviceScannerV4 = new DeviceScannerV4(
      tenant_id, device_id,
      13,                                           // 1: maxChannel (int)
      (unsigned long)1000,                          // 2: channelInterval (unsigned long)
      (uint32_t)3,                                  // 3: bleScanDuration (uint32_t)
      (unsigned long)5000,                          // 4: ttlCheckInterval (unsigned long)
      (unsigned long)device_ttl,                    // 5: ttlInterval (unsigned long, CASTED from int)
      (uint16_t)15,                                 // 6: maxWiFiDevices (uint16_t)
      (uint16_t)15,                                 // 7: maxBLEDevices (uint16_t)
      savedSSID, savedPassword,                     // 8, 9: std::string
      mqttServer, (uint16_t)mqttPort,               // 10, 11: std::string, uint16_t (CASTED from int)
      mqttUsername, mqttPassword,                   // 12, 13: std::string
      TOPIC_DATA, (unsigned long)publish_interval,  // 14, 15: std::string, unsigned long (CASTED from int)
      rssi_filter,                                  // 16 rss_filter
      country);                                     // 17 rss_filter
                                                    // The rest (16, 17) are default arguments or not explicitly passed.
    // Note: The previous call was 18 arguments. We removed the extra '3000' argument.
    // The total is 15 arguments explicitly passed, matching the non-default arguments in your previous calls.

    deviceScannerV4->begin();

    return true;
  }
}

/**
  * Clear all stored configuration settings.
  */
void clearConfig() {
  tenant_id = "";
  device_id = "";
  savedSSID = "";
  savedPassword = "";
  mqttServer = "";
  mqttPort = 1883;
  mqttUsername = "";
  mqttPassword = "";

  TOPIC_DATA = "";
  TOPIC_HEALTH = "";

  rssi_filter = -100;
  geofence_radius = 8;

  scan_interval = 10000;
  publish_interval = 15000;
  device_ttl = 10000;

  isConfigured = false;
  // FIX (Confirmation of existing logic): Ensure object is deleted and pointer zeroed
  if (deviceScannerV4) {
    delete deviceScannerV4;  // Calls the new destructor to cleanup BLE callback!
    deviceScannerV4 = nullptr;
  }
}

// Utility functions

/**
  * Helper function to create a JSON payload for event data.
  * @param doc The JSON document to which the data will be added.
  * @param eventType Type of event (e.g., "warning", "alert"). (REPLACED String)
  * @param description Description of the event. (REPLACED String)
  * @param severity Severity level (e.g., "high", "low"). (REPLACED String)
  * @param actualValue Optional actual value associated with the event. (REPLACED String)
  * @param alertMessage Optional alert message related to the event. (REPLACED String)
  * @return The created JSON object.
  */
JsonObject createJsonData(JsonDocument& doc,
                          std::string eventType,
                          std::string description,
                          std::string severity,
                          std::string actualValue = "",
                          std::string alertMessage = "") {
  JsonObject eventNode = doc.createNestedObject("e");
  eventNode["tenant_id"] = tenant_id;
  eventNode["serial_number"] = device_id;
  eventNode["event_type"] = eventType;
  eventNode["description"] = description;
  eventNode["severity"] = severity;

  // Add optional fields if provided. Use .empty()
  if (!actualValue.empty()) {
    eventNode["actual_value"] = actualValue;
  }
  if (!alertMessage.empty()) {
    eventNode["alert_message"] = alertMessage;
  }

  return eventNode;
}

// Function arguments remain const char* as they are low-level C-functions (WiFi/MQTT)
bool connectToWiFi(const char* ssid, const char* password, const char* tenant_id) {
  // Serial.print("Connecting to WiFi...");

  setWifiCountryByString(country.c_str());

  WiFi.begin(ssid, password);

  int attempt = 0;
  const int maxAttempts = 5;
  const int baseDelay = 600;  // Base delay in ms

  while (WiFi.status() != WL_CONNECTED && attempt < maxAttempts && !Serial.available() > 0) {
    int backoffDelay = (attempt < 5) ? baseDelay * (attempt + 1) : baseDelay * 6;  // Linear backoff: 1000ms, 2000ms, ...upto 5 attempts after that fixed attempts
    delay(backoffDelay);
    attempt++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    //Serial.println("Connected to WiFi");
    return true;
  } else {
    Serial.println("\nFailed to connect to WiFi.");
    return false;
  }
}

// Function arguments remain const char*
bool connectToMqtt(const char* mqttServer, int mqttPort, const char* mqttUsername, const char* mqttPassword) {
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);

  int attempt = 0;
  const int maxAttempts = 5;
  const int baseDelay = 600;  // Base delay in ms

  while (!mqttClient.connected() && attempt < maxAttempts && !Serial.available() > 0) {
    // Use device_id.c_str()
    if (mqttClient.connect(device_id.c_str(), mqttUsername, mqttPassword)) {
      return true;
    } else {
      int backoffDelay = (attempt < 5) ? baseDelay * (attempt + 1) : baseDelay * 6;  // Linear backoff: 1000ms, 2000ms, ...upto 5 attempts after that fixed attempts
      delay(backoffDelay);
      attempt++;
    }
  }

  // Serial.println("Failed to connect to MQTT after multiple attempts.");
  return false;
}

/**
  * Generate a JSON payload with system health data (e.g., uptime).
  * @return JSON-formatted health data as a std::string. (REPLACED String)
  */
std::string getHealthData() {
  unsigned long uptime = millis() / 1000;  // Get uptime in seconds
  std::string payload = "{";
  payload += "\"status\":\"active\",";
  // Use std::to_string for number to string conversion
  payload += "\"uptime_seconds\": " + std::to_string(uptime);
  payload += "}";

  return payload;
}


/**
  * Helper function to publish data to MQTT broker. Ensures WiFi and MQTT connectivity.
  * @param topic MQTT topic to publish to. (REPLACED String)
  * @param payload Data to be sent in the message. (REPLACED String)
  * @return True if data was published successfully, false if failed.
  */
bool publishData(const std::string& topic, const std::string& payload) {
  // Ensure WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    // Serial.println("WiFi disconnected. Attempting to reconnect...");
    // Use .c_str() to convert std::string to const char*
    if (!connectToWiFi(savedSSID.c_str(), savedPassword.c_str(), tenant_id.c_str())) {
      // Serial.println("Failed to reconnect to WiFi.");
      return false;
    }
  }

  // Ensure MQTT is connected
  if (!mqttClient.connected()) {
    // Use .c_str()
    if (!connectToMqtt(mqttServer.c_str(), mqttPort, mqttUsername.c_str(), mqttPassword.c_str())) {
      // Serial.println("Failed to reconnect to MQTT broker.");
      return false;
    }
  }

  // Publish data to the specified MQTT topic. Use .c_str()
  if (mqttClient.publish(topic.c_str(), payload.c_str())) {
    return true;
  } else {
    Serial.println("Failed to publish data.");
    return false;
  }
}


void setup() {
  Serial.begin(115200);
  Serial.setRxBufferSize(SERIAL_BUFFER_SIZE);

  // Standard guard for boards with native USB Serial
  if (Serial)
    while (!Serial) {
      delay(1);
    }

  // Set CPU frequency for better performance/power balance
  // setCpuFrequencyMhz(160);

  isConfigured = loadConfig();  // Load configuration
}

void loop() {
  // Handle serial commands (e.g., reset config)
  if (Serial.available() > 0) {
    // Read serial command as an Arduino String first, then convert to std::string
    std::string commandString = Serial.readStringUntil('\n').c_str();
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
    deviceScannerV4->update();

    // Health monitor
    unsigned long currentMillis = millis();
    if (currentMillis - lastSentTime >= HEALTH_PUBLISH_INTERVAL) {
      std::string healthData = getHealthData();  // Get health data
      // Use .c_str() for the topic and payload
      if (!publishData(TOPIC_HEALTH, healthData)) {
        Serial.println("Failed to publish health data");
      }
      lastSentTime = currentMillis;  // Update last sent time
      healthData.clear();
    }
  }
}