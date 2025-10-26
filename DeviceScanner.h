#ifndef DEVICE_SCANNER_V4_H
#define DEVICE_SCANNER_V4_H

#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <limits.h>
#include <string>
#include <sstream>
#include <cmath>
#include <cstring>
#include "esp_wifi.h"

// --- Configuration ---
#define DEBUG false
#define MQTT_BUFFER_SIZE 8192
#define CORRELATION_TTL_INTERVAL (2 * 60 * 1000)  // 2 minutes
#define MANUFACTURER "aap-v2-scanner"

// --- Wi-Fi Promiscuous Packet Structures ---
struct MacHeader {
  uint16_t frame_control;
  uint16_t duration_id;
  uint8_t addr1[6];
  uint8_t addr2[6];
  uint8_t addr3[6];
  uint16_t sequence_control;
  uint8_t addr4[6];
};

// =========================================================================
// --- Device Info Structure (MAX MEMORY OPTIMIZED) ---
// =========================================================================
struct DeviceInfo {
  // CRITICAL OPTIMIZATION: Use uint64_t (8 bytes) for MAC key, const char* for status ("A", "R", "B")
  // status: "R" for Randomized, "A" for Actual/Static, "B" for BLE
  std::map<uint64_t, const char*> macs;

  const char* type;
  const char* manufacturer;
  char name[33];
  unsigned long timestamp;
  uint32_t correlatedID;
  int rssi = -120;
};

// --- Forward declaration for global instance ---
class DeviceScannerV4;
extern DeviceScannerV4* deviceScannerV4;

// --- Enum for State Machine ---
enum ScannerState {
  STATE_INIT,
  STATE_WIFI_PROMISCUOUS_SCAN,
  STATE_BLE_SCAN,
  STATE_MQTT_PUBLISH,
  STATE_IDLE
};

// =========================================================================
// --- MAC/ID CONVERSION HELPERS (Inline) ---
// =========================================================================

inline uint64_t macToUint64(uint8_t* mac) {
  uint64_t result = 0;
  for (int i = 0; i < 6; i++) {
    result = (result << 8) | mac[i];
  }
  return result;
}

inline std::string uint64ToMacString(uint64_t mac) {
  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          (uint8_t)((mac >> 40) & 0xFF), (uint8_t)((mac >> 32) & 0xFF),
          (uint8_t)((mac >> 24) & 0xFF), (uint8_t)((mac >> 16) & 0xFF),
          (uint8_t)((mac >> 8) & 0xFF), (uint8_t)(mac & 0xFF));
  return std::string(buf);
}

inline const char* getManufacturerFromOUI_ptr(const std::string& oui) {
  if (oui == "F8:E7:1E") return "GOOGLE";
  if (oui == "EC:F4:51" || oui == "C8:85:47" || (oui[0] >= '9' && oui[0] <= 'D' && oui[1] == '8')) return "APPLE";  // Broad Apple OUI range
  if (oui == "BC:D1:D1") return "HUAWEI";
  if (oui == "54:77:8E") return "XIAOMI";
  return "Unknown";
}


// =========================================================================
// --- Correlation Engine (NUMERICAL ID) ---
// =========================================================================

class CorrelationEngine {
public:
  uint32_t getCorrelationID(DeviceInfo& info);
  void purgeOldCorrelations(unsigned long now);

private:
  std::map<uint32_t, std::set<uint64_t>> mac_history;
  std::map<uint64_t, uint32_t> mac_to_id;
  std::map<std::string, uint32_t> name_to_id;
  std::map<uint32_t, unsigned long> last_seen_id;

  uint32_t next_correlation_id = 1;
};

// --- Correlation Engine Implementations (Inline) ---
inline uint32_t CorrelationEngine::getCorrelationID(DeviceInfo& info) {
  uint32_t found_id_num = 0;
  const char* device_name = info.name;

  // 1. Check for existing ID based on any MAC address (uint64_t key)
  for (auto const& [mac_num, status_ptr] : info.macs) {
    if (mac_to_id.count(mac_num)) {
      found_id_num = mac_to_id.at(mac_num);
      break;
    }
  }

  // 2. Check for existing ID based on device name (std::string key)
  std::string name_str(device_name);
  if (found_id_num == 0 && name_str.length() > 0 && name_str.find("Unknown") == std::string::npos) {
    if (name_to_id.count(name_str)) {
      found_id_num = name_to_id.at(name_str);
    }
  }

  // 3. Generate a new ID if none found
  if (found_id_num == 0) {
    found_id_num = next_correlation_id++;
  }

  // 4. Update DeviceInfo and History
  info.correlatedID = found_id_num;
  last_seen_id[found_id_num] = millis();

  for (auto const& [mac_num, status_ptr] : info.macs) {
    if (mac_num != 0 && strcmp(status_ptr, "A") == 0) {  // Only use non-randomized MACs for long-term correlation history
      mac_history[found_id_num].insert(mac_num);
      mac_to_id[mac_num] = found_id_num;
    }
  }

  if (name_str.length() > 0 && name_str.find("Unknown") == std::string::npos) {
    name_to_id[name_str] = found_id_num;
  }
  return found_id_num;
}

inline void CorrelationEngine::purgeOldCorrelations(unsigned long now) {
  for (auto it = last_seen_id.begin(); it != last_seen_id.end();) {
    if (now - it->second >= CORRELATION_TTL_INTERVAL) {
      uint32_t id_to_remove = it->first;

      if (mac_history.count(id_to_remove)) {
        for (const uint64_t& mac_num : mac_history.at(id_to_remove)) {
          mac_to_id.erase(mac_num);
        }
        mac_history.erase(id_to_remove);
      }

      for (auto name_it = name_to_id.begin(); name_it != name_to_id.end();) {
        if (name_it->second == id_to_remove) {
          name_it = name_to_id.erase(name_it);
        } else {
          ++name_it;
        }
      }
      it = last_seen_id.erase(it);
    } else {
      ++it;
    }
  }
}

// --- BLE Scan Callback Handler ---
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
public:
  MyAdvertisedDeviceCallbacks(DeviceScannerV4* scanner)
    : _scanner(scanner) {}
  void onResult(BLEAdvertisedDevice advertisedDevice) override;
private:
  DeviceScannerV4* _scanner;
};


// =========================================================================
// --- Device Scanner Class V4 (Header) ---
// =========================================================================
class DeviceScannerV4 {
public:
  // Constructor declaration
  DeviceScannerV4(
    const std::string tenant_id, const std::string device_id, int maxChannel = 13,
    unsigned long channelInterval = 1000, uint32_t bleScanDuration = 2,
    unsigned long ttlCheckInterval = 60000, unsigned long ttlInterval = 180000,
    uint16_t maxWiFiDevices = 100, uint16_t maxBLEDevices = 100,
    const std::string& ssid = "", const std::string& password = "",
    const std::string& mqttServer = "", uint16_t mqttPort = 1883,
    const std::string& mqttUsername = "", const std::string& mqttPassword = "",
    const std::string& mqttTopic = "devices/data", unsigned long mqttPublishInterval = 10000);

  // Destructor declaration (CRITICAL FIX)
  ~DeviceScannerV4();

  void begin();
  void update();

  static void wifiPacketHandlerStatic(void* buffer, wifi_promiscuous_pkt_type_t type);

  // Prototypes for functions defined externally/later
  void processBLEAdvertisedDevice_impl(BLEAdvertisedDevice advertisedDevice);
  void printDiscoveredDevices();
  std::string getDevicesGroupedByTypeAsJson(uint8_t maxWifi, uint8_t maxBle);

  // CRITICAL: External implementation required (in .ino or .cpp)
  bool connectToWiFi() {
    if (WiFi.status() == WL_CONNECTED) return true;
    if (ssid.empty() || password.empty()) {
      if (DEBUG && Serial) Serial.println("[WIFI DEBUG] Credentials are empty!");
      return false;
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    if (DEBUG && Serial) Serial.printf("[WIFI DEBUG] Attempting to connect to: %s\n", ssid.c_str());

    int attempts = 0;
    int baseDelay = 1000;
    while (WiFi.status() != WL_CONNECTED && attempts < 15) {
      if (attempts < 5) {
        delay(baseDelay * (attempts + 1));
      } else {
        delay(baseDelay * 6);
      }
      if (DEBUG && Serial) Serial.printf(".");
      attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
      if (DEBUG && Serial) Serial.printf("\n[WIFI DEBUG] Failed after %d attempts. FINAL Status Code: %d\n", attempts, WiFi.status());
      delay(10);
    }
    return WiFi.status() == WL_CONNECTED;
  }

  bool connectToMqtt() {
    if (mqttClient.connected()) return true;
    mqttClient.setServer(mqttServer.c_str(), mqttPort);
    mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
    int attempts = 0;
    int baseDelay = 1000;
    while (!mqttClient.connected() && attempts < 5) {
      if (mqttClient.connect("AAP-V2", mqttUsername.c_str(), mqttPassword.c_str())) {
        return true;
      }
      if (attempts < 5) {
        delay(baseDelay * (attempts + 1));
      } else {
        delay(baseDelay * 6);
      }
      attempts++;
    }
    return false;
  }

private:
  // CRITICAL FIX: Store the pointer to the dynamically allocated callback object
  BLEAdvertisedDeviceCallbacks* pAdvertisedDeviceCallbacks = nullptr;

  std::string tenant_id;
  std::string device_id;

  ScannerState current_state = STATE_INIT;
  unsigned long previousStateChangeMillis = 0;

  // Map stores devices keyed by their numerical correlation ID
  std::map<uint32_t, DeviceInfo> discovered_devices;
  // Lookup map to find the numerical ID using the original string key (SSID/MAC/Name)
  std::map<std::string, uint32_t> key_to_id_lookup;

  CorrelationEngine correlationEngine;

  // --- Configuration Variables ---
  const uint16_t maxWiFiDevices;
  const uint16_t maxBLEDevices;
  const uint32_t bleScanDuration;
  unsigned long previousTTLCheckMillis = 0;
  const unsigned long ttlCheckInterval;
  const unsigned long ttlInterval;
  int currentChannel = 1;
  const int maxChannel;
  unsigned long previousChannelMillis = 0;
  const unsigned long channelInterval;

  std::string ssid;
  std::string password;
  std::string mqttServer;
  uint16_t mqttPort;
  std::string mqttUsername;
  std::string mqttPassword;
  std::string mqttTopic;
  unsigned long mqttPublishInterval;
  unsigned long previousMqttPublishMillis = 0;

  WiFiClient espClient;
  PubSubClient mqttClient;
  BLEScan* pBLEScan = nullptr;

  // --- Private Member Function Implementations (Inline) ---
  void startPromiscuousScan() {
    if (DEBUG && Serial)
      Serial.println("[Wi-Fi Mode Switch] Starting Promiscuous Scan (AP + Client)...");

    WiFi.disconnect(true);
    WiFi.mode(WIFI_MODE_STA);
    esp_wifi_set_promiscuous_rx_cb(wifiPacketHandlerStatic);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  }

  void stopPromiscuousScan() {
    if (DEBUG && Serial)
      Serial.println("[Wi-Fi Mode Switch] Stopping Promiscuous Scan...");
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(NULL);
    WiFi.mode(WIFI_MODE_NULL);
    delay(50);
  }

  void channelHop() {
    bool is_promiscuous_enabled = false;
    esp_wifi_get_promiscuous(&is_promiscuous_enabled);

    if (is_promiscuous_enabled) {
      esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
      currentChannel++;
      if (currentChannel > maxChannel) {
        currentChannel = 1;
      }
    }
  }

  void wifiPacketHandler(void* buffer, wifi_promiscuous_pkt_type_t type);
  void sendDataToMqtt();
  void removeExpiredEntries();

  // Helper function implementations (Inline)
  std::string macToString(uint8_t* mac);
  std::string getOUI(uint8_t* mac);
  bool isRandomizedMac(uint8_t* mac);
  uint16_t getWiFiDeviceCount() const;
  uint16_t getBLEDeviceCount() const;
};

// =========================================================================
// --- Implementation of External/Non-Inline Functions ---
// =========================================================================

// --- Constructor Implementation ---
DeviceScannerV4::DeviceScannerV4(
  const std::string tenant_id, const std::string device_id, int maxChannel,
  unsigned long channelInterval, uint32_t bleScanDuration,
  unsigned long ttlCheckInterval, unsigned long ttlInterval,
  uint16_t maxWiFiDevices, uint16_t maxBLEDevices,
  const std::string& ssid, const std::string& password,
  const std::string& mqttServer, uint16_t mqttPort,
  const std::string& mqttUsername, const std::string& mqttPassword,
  const std::string& mqttTopic, unsigned long mqttPublishInterval)
  : tenant_id(tenant_id), device_id(device_id), channelInterval(channelInterval),
    bleScanDuration(bleScanDuration), ttlCheckInterval(ttlCheckInterval),
    ttlInterval(ttlInterval), maxWiFiDevices(maxWiFiDevices),
    maxBLEDevices(maxBLEDevices), maxChannel(maxChannel), ssid(ssid),
    password(password), mqttServer(mqttServer), mqttPort(mqttPort),
    mqttUsername(mqttUsername), mqttPassword(mqttPassword), mqttTopic(mqttTopic),
    mqttPublishInterval(mqttPublishInterval), mqttClient(espClient) {}

// --- Destructor Implementation (CRITICAL FIX) ---
DeviceScannerV4::~DeviceScannerV4() {
  // 1. Clean up the dynamically allocated BLE callback object
  if (pAdvertisedDeviceCallbacks) {
    delete pAdvertisedDeviceCallbacks;
    pAdvertisedDeviceCallbacks = nullptr;
  }
  // 2. Stop BLE scanning and WiFi promiscuous mode
  if (pBLEScan) {
    pBLEScan->stop();
  }
  stopPromiscuousScan();  // Ensure promiscuous mode is off
}
// --- begin Implementation (CRITICAL FIX: Store the callback pointer) ---
void DeviceScannerV4::begin() {
  Serial.begin(115200);
  delay(100);

  BLEDevice::init(MANUFACTURER);
  pBLEScan = BLEDevice::getScan();

  // Allocate the callback and store the pointer
  pAdvertisedDeviceCallbacks = new MyAdvertisedDeviceCallbacks(this);
  pBLEScan->setAdvertisedDeviceCallbacks(pAdvertisedDeviceCallbacks);

  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(100);

  if (DEBUG && Serial) Serial.println("AAP V2 Client/AP/BLE Scanner V2 Started (Max Optimized)");

  startPromiscuousScan();
  current_state = STATE_WIFI_PROMISCUOUS_SCAN;
}

// This is the definition for the static function.
void DeviceScannerV4::wifiPacketHandlerStatic(void* buffer, wifi_promiscuous_pkt_type_t type) {
  if (deviceScannerV4) {
    deviceScannerV4->wifiPacketHandler(buffer, type);
  }
}
// --- update Implementation ---
void DeviceScannerV4::update() {
  unsigned long currentMillis = millis();

  // 1. TTL Check (Always running)
  if (currentMillis - previousTTLCheckMillis >= ttlCheckInterval) {
    previousTTLCheckMillis = currentMillis;
    removeExpiredEntries();
    correlationEngine.purgeOldCorrelations(currentMillis);
    if (DEBUG) printDiscoveredDevices();  // Print only if DEBUG is true
  }

  // 2. State Machine Logic
  if (current_state == STATE_WIFI_PROMISCUOUS_SCAN) {
    if (currentMillis - previousChannelMillis >= channelInterval) {
      previousChannelMillis = currentMillis;
      channelHop();
    }
    if (currentMillis - previousStateChangeMillis >= 3000) {
      if (DEBUG && Serial) Serial.println("[TRANSITION] Promiscuous cycle end. -> STATE_BLE_SCAN");
      stopPromiscuousScan();
      current_state = STATE_BLE_SCAN;
      previousStateChangeMillis = currentMillis;
    }
  }

  switch (current_state) {
    case STATE_WIFI_PROMISCUOUS_SCAN: break;
    case STATE_BLE_SCAN:
      if (DEBUG && Serial) Serial.println("\n[STATE] Starting BLE Scan...");
      WiFi.mode(WIFI_MODE_NULL);
      pBLEScan->start(bleScanDuration, false);
      pBLEScan->stop();
      if (DEBUG && Serial) Serial.println("[TRANSITION] BLE Scan end. -> STATE_MQTT_PUBLISH");
      current_state = STATE_MQTT_PUBLISH;
      previousStateChangeMillis = currentMillis;
      break;
    case STATE_MQTT_PUBLISH:
      if (currentMillis - previousMqttPublishMillis >= mqttPublishInterval) {
        previousMqttPublishMillis = currentMillis;
        sendDataToMqtt();
      } else {
        if (DEBUG && Serial) Serial.printf("[MQTT INFO] Skipping publish. Next in %lu ms.\n", mqttPublishInterval - (currentMillis - previousMqttPublishMillis));
      }
      current_state = STATE_IDLE;
      previousStateChangeMillis = currentMillis;
      break;
    case STATE_IDLE:
      if (currentMillis - previousStateChangeMillis >= 1000) {
        if (DEBUG && Serial) Serial.println("[TRANSITION] Idle end. -> STATE_WIFI_PROMISCUOUS_SCAN");
        startPromiscuousScan();
        current_state = STATE_WIFI_PROMISCUOUS_SCAN;
        previousStateChangeMillis = currentMillis;
      }
      break;
    default: current_state = STATE_WIFI_PROMISCUOUS_SCAN; break;
  }
  mqttClient.loop();
}

// --- Helper function implementations (Defined outside class, using ::) ---
std::string DeviceScannerV4::macToString(uint8_t* mac) {
  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return std::string(buf);
}

std::string DeviceScannerV4::getOUI(uint8_t* mac) {
  char buf[9];
  sprintf(buf, "%02X:%02X:%02X", mac[0], mac[1], mac[2]);
  return std::string(buf);
}

bool DeviceScannerV4::isRandomizedMac(uint8_t* mac) {
  return (mac[0] & 0x02) == 0x02;
}

uint16_t DeviceScannerV4::getWiFiDeviceCount() const {
  uint16_t cnt = 0;
  for (auto const& kv : discovered_devices) {
    if (strcmp(kv.second.type, "Wi-Fi AP") == 0 || strcmp(kv.second.type, "Wi-Fi Client") == 0) cnt++;
  }
  return cnt;
}

uint16_t DeviceScannerV4::getBLEDeviceCount() const {
  uint16_t cnt = 0;
  for (auto const& kv : discovered_devices) {
    if (strcmp(kv.second.type, "BLE") == 0) cnt++;
  }
  return cnt;
}

void DeviceScannerV4::removeExpiredEntries() {
  unsigned long now = millis();
  for (auto it = discovered_devices.begin(); it != discovered_devices.end();) {
    if (now - it->second.timestamp >= ttlInterval) {

      uint32_t id_to_remove = it->first;

      // Remove from string lookup table
      for (auto lookup_it = key_to_id_lookup.begin(); lookup_it != key_to_id_lookup.end();) {
        if (lookup_it->second == id_to_remove) {
          lookup_it = key_to_id_lookup.erase(lookup_it);
        } else {
          ++lookup_it;
        }
      }

      it = discovered_devices.erase(it);
    } else {
      ++it;
    }
  }
}


// --- sendDataToMqtt Implementation ---
// --- sendDataToMqtt Implementation ---
void DeviceScannerV4::sendDataToMqtt() {
  if (DEBUG && Serial) Serial.println("[MQTT] Starting publish cycle (Inside function)...");

  if (!connectToWiFi()) {
    if (DEBUG && Serial) Serial.println("[MQTT FAILED] ❌ Wi-Fi Connection failed. Skipping MQTT.");
    return;
  }
  if (DEBUG && Serial) Serial.println("[MQTT SUCCESS] ✅ Wi-Fi Connected.");

  if (!connectToMqtt()) {
    if (DEBUG && Serial) Serial.println("[MQTT FAILED] ❌ MQTT Broker connection failed. Skipping publish.");
    WiFi.disconnect(true);
    return;
  }
  if (DEBUG && Serial) Serial.println("[MQTT SUCCESS] ✅ MQTT Broker Connected.");

  std::string jsonData = getDevicesGroupedByTypeAsJson(maxWiFiDevices, maxBLEDevices);

  if (jsonData.empty()) {
    if (DEBUG && Serial) Serial.println("[MQTT INFO] JSON data is empty or serialization failed.");
  } else {
    if (DEBUG && Serial) Serial.printf("[MQTT] Publishing %zu bytes to topic '%s'...\n", jsonData.length(), mqttTopic.c_str());

    if (mqttClient.publish(mqttTopic.c_str(), jsonData.c_str())) {
      if (DEBUG && Serial) Serial.println("[MQTT SUCCESS] 🎉 Data published successfully.");
    } else {
      if (DEBUG && Serial) Serial.printf("[MQTT FAILED] ❌ Publish command failed. Client state: %d\n", mqttClient.state());
    }
  }

  // Allow MQTT client to process a few loops before disconnecting
  for (int i = 0; i < 20; i++) {
    mqttClient.loop();
    delay(10);
  }

  // Free up heap memory aggressively after publishing
  jsonData.clear();

  // CRITICAL FIX: Only clear the transient/scanned data.
  // discovered_devices holds the recently scanned data that was just published.
  discovered_devices.clear();

  // DO NOT CLEAR key_to_id_lookup! It is required for fast lookups in the
  // next scan cycle and is part of the correlation bridge to the CorrelationEngine.

  WiFi.disconnect(true);
  if (DEBUG && Serial) 
    Serial.println("[MQTT] Publishing cycle finished. Wi-Fi disconnected.");
}

// --- Wi-Fi Packet Handler Implementation (Refactored Name Logic) ---
void DeviceScannerV4::wifiPacketHandler(void* buffer, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;

  wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buffer;
  uint8_t* payload = pkt->payload;
  if (pkt->rx_ctrl.sig_len < sizeof(MacHeader)) return;

  MacHeader* macHeader = (MacHeader*)payload;
  uint8_t frame_control = macHeader->frame_control & 0xFF;
  uint8_t subtype = (frame_control >> 4) & 0x0F;
  uint8_t type_val = (frame_control >> 2) & 0x03;

  const char* type_label_ptr = nullptr;
  uint8_t* source_mac_ptr = macHeader->addr2;

  // Determine Type
  if (type_val == 0) {
    if (subtype == 8) {
      type_label_ptr = "Wi-Fi AP";
    } else if (subtype == 4) {
      type_label_ptr = "Wi-Fi Client";
    } else {
      return;
    }
  } else {
    return;
  }

  uint64_t mac_num = macToUint64(source_mac_ptr);
  if (mac_num == 0) return;

  std::string mac_address_str = macToString(source_mac_ptr);
  std::string oui = getOUI(source_mac_ptr);
  int rssi = pkt->rx_ctrl.rssi;
  bool is_randomized = isRandomizedMac(source_mac_ptr);

  const char* manufacturer_ptr = getManufacturerFromOUI_ptr(oui);
  std::string deviceName;
  // Status is "R" for randomized, "A" for actual/static
  const char* mac_status_ptr = is_randomized ? "R" : "A";

  // Determine Device Name (Refactored: Removed suffixes)
  if (type_label_ptr == "Wi-Fi AP") {
    if (pkt->rx_ctrl.sig_len > 38 && payload[36] == 0) {
      uint8_t ssid_len = payload[37];
      if (ssid_len > 0 && ssid_len <= 32) {
        char ssid_buf[33];
        memcpy(ssid_buf, &payload[38], ssid_len);
        ssid_buf[ssid_len] = '\0';
        deviceName = std::string(ssid_buf);
      } else {
        deviceName = mac_address_str;  // Use MAC for hidden/unknown AP
      }
    } else {
      deviceName = mac_address_str;  // Use MAC for unknown AP
    }
  } else {  // Wi-Fi Client
    // Use MAC as the primary name for clients
    deviceName = mac_address_str;
  }

  // --- Data Storage Logic (Using lookup table) ---
  std::string string_key = deviceName;
  if (type_label_ptr == "Wi-Fi Client") {
    string_key = mac_address_str;
  }

  uint32_t numerical_id = 0;

  auto id_it = key_to_id_lookup.find(string_key);
  if (id_it != key_to_id_lookup.end()) {
    numerical_id = id_it->second;

    auto dev_it = discovered_devices.find(numerical_id);
    if (dev_it != discovered_devices.end()) {
      DeviceInfo& existing = dev_it->second;
      existing.timestamp = millis();
      if (rssi > existing.rssi) {
        existing.rssi = rssi;
      }
      existing.macs[mac_num] = mac_status_ptr;
      correlationEngine.getCorrelationID(existing);
      return;
    }
  }

  // New Device Logic
  if (getWiFiDeviceCount() >= maxWiFiDevices) return;

  DeviceInfo newDev;
  newDev.macs[mac_num] = mac_status_ptr;
  newDev.type = type_label_ptr;
  newDev.manufacturer = manufacturer_ptr;
  strncpy(newDev.name, deviceName.c_str(), 32);
  newDev.name[32] = '\0';
  newDev.timestamp = millis();
  newDev.rssi = rssi;

  uint32_t final_id = correlationEngine.getCorrelationID(newDev);

  discovered_devices[final_id] = newDev;
  key_to_id_lookup[string_key] = final_id;
}


// --- processBLEAdvertisedDevice_impl implementation (Refactored Name Logic) ---
void MyAdvertisedDeviceCallbacks::onResult(BLEAdvertisedDevice advertisedDevice) {
  if (deviceScannerV4) {
    deviceScannerV4->processBLEAdvertisedDevice_impl(advertisedDevice);
  }
}

void DeviceScannerV4::processBLEAdvertisedDevice_impl(BLEAdvertisedDevice advertisedDevice) {
  std::string temp_device_name;

  if (advertisedDevice.haveName()) {
    temp_device_name = advertisedDevice.getName().c_str();
  } else {
    temp_device_name = advertisedDevice.getAddress().toString().c_str();
  }

  if (getBLEDeviceCount() >= maxBLEDevices) return;

  uint8_t* mac_bytes = advertisedDevice.getAddress().getNative();
  std::string oui = getOUI(mac_bytes);
  const char* manufacturer_ptr = getManufacturerFromOUI_ptr(oui);
  uint64_t mac_num = macToUint64(mac_bytes);
  const char* mac_status_ptr = "B";

  // --- Data Storage Logic (Using lookup table) ---
  std::string string_key = temp_device_name;
  uint32_t numerical_id = 0;

  auto id_it = key_to_id_lookup.find(string_key);
  if (id_it != key_to_id_lookup.end()) {
    numerical_id = id_it->second;

    auto dev_it = discovered_devices.find(numerical_id);
    if (dev_it != discovered_devices.end()) {
      DeviceInfo& existing = dev_it->second;
      existing.timestamp = millis();
      if (advertisedDevice.getRSSI() > existing.rssi) {
        existing.rssi = advertisedDevice.getRSSI();
      }
      existing.macs[mac_num] = mac_status_ptr;
      correlationEngine.getCorrelationID(existing);
      return;
    }
  }

  // New Device Logic
  DeviceInfo newDev;
  newDev.macs[mac_num] = mac_status_ptr;
  newDev.type = "BLE";
  newDev.manufacturer = manufacturer_ptr;
  strncpy(newDev.name, temp_device_name.c_str(), 32);
  newDev.name[32] = '\0';
  newDev.timestamp = millis();
  newDev.rssi = advertisedDevice.getRSSI();

  uint32_t final_id = correlationEngine.getCorrelationID(newDev);

  discovered_devices[final_id] = newDev;
  key_to_id_lookup[string_key] = final_id;
}


// --- Print function implementation (Updated for MAC formatting) ---
void DeviceScannerV4::printDiscoveredDevices() {
  if (!Serial) return;
  if (DEBUG && Serial) Serial.println("--- Current Device Cache V4 (MAX OPTIMIZED) ---");

  std::map<uint32_t, std::vector<const DeviceInfo*>> grouped;
  for (auto const& [id, info] : discovered_devices) {
    grouped[info.correlatedID].push_back(&info);
  }

  std::vector<std::pair<uint32_t, std::vector<const DeviceInfo*>>> sortedGroups(grouped.begin(), grouped.end());
  std::sort(sortedGroups.begin(), sortedGroups.end(),
            [](auto const& a, auto const& b) {
              int maxRssiA = INT_MIN;
              for (auto const& dev : a.second) {
                if (dev->rssi > maxRssiA) maxRssiA = dev->rssi;
              }
              int maxRssiB = INT_MIN;
              for (auto const& dev : b.second) {
                if (dev->rssi > maxRssiB) maxRssiB = dev->rssi;
              }
              return maxRssiA > maxRssiB;
            });

  for (auto const& grp : sortedGroups) {
    uint32_t correlatedID_num = grp.first;
    const std::vector<const DeviceInfo*>& devices = grp.second;
    if (devices.empty()) continue;

    const DeviceInfo* strongestDev = devices[0];
    int strongestRSSI = INT_MIN;
    unsigned long age = (millis() - strongestDev->timestamp) / 1000;

    std::set<std::string> all_formatted_macs;

    for (auto const& dev : devices) {
      for (auto const& [mac_num, status_ptr] : dev->macs) {
        // Add (*) only if randomized ("R")
        std::string mac_str = uint64ToMacString(mac_num);
        if (strcmp(status_ptr, "R") == 0) {
          mac_str += "*";
        }
        all_formatted_macs.insert(mac_str);
      }
      if (dev->rssi > strongestRSSI) {
        strongestRSSI = dev->rssi;
      }
    }

    std::string macsStr;
    for (const auto& formatted_mac : all_formatted_macs) {
      macsStr += formatted_mac + " ";
    }
    if (!macsStr.empty() && macsStr.back() == ' ') {
      macsStr.pop_back();
    }

    std::string id_str = "Device_" + std::to_string(correlatedID_num);

    if (DEBUG && Serial)
      Serial.printf(
        "ID: %s | Type: %s | Name: %s | RSSI: %d | MACs: %s | Age: %lus\n",
        id_str.c_str(),
        strongestDev->type,
        strongestDev->name,
        strongestRSSI,
        macsStr.c_str(),
        age);
  }
  if (DEBUG && Serial) Serial.println("-------------------------------------------------");
  if (DEBUG && Serial) Serial.printf("Total Wi-Fi Devices: %d, Total BLE Devices: %d\n", getWiFiDeviceCount(), getBLEDeviceCount());
}

// --- JSON function implementation (Target Payload Structure) ---
// Note: This assumes the header and DeviceInfo structure from the previous context.
std::string DeviceScannerV4::getDevicesGroupedByTypeAsJson(uint8_t maxWifi, uint8_t maxBle) {
  // Use a generous buffer size (e.g., 4096 or MQTT_BUFFER_SIZE)
  DynamicJsonDocument doc(MQTT_BUFFER_SIZE); 
  
  // 1. Top-Level Field
  doc["device_id"] = device_id;

  JsonArray wifiArray = doc.createNestedArray("wifi");
  JsonArray bleArray = doc.createNestedArray("ble");

  // --- Aggregation and Sorting Setup (Kept for correctness) ---
  // Group devices by their correlation ID
  std::map<uint32_t, std::vector<const DeviceInfo*>> grouped;
  for (auto const& [id, info] : discovered_devices) {
    grouped[info.correlatedID].push_back(&info);
  }

  // Sort groups by strongest RSSI
  std::vector<std::pair<uint32_t, std::vector<const DeviceInfo*>>> sortedGroups(grouped.begin(), grouped.end());
  std::sort(sortedGroups.begin(), sortedGroups.end(),
            [](auto const& a, auto const& b) {
              int maxRssiA = INT_MIN;
              for (auto const& dev : a.second) {
                if (dev->rssi > maxRssiA) maxRssiA = dev->rssi;
              }
              int maxRssiB = INT_MIN;
              for (auto const& dev : b.second) {
                if (dev->rssi > maxRssiB) maxRssiB = dev->rssi;
              }
              return maxRssiA > maxRssiB;
            });

  uint8_t local_wifiCount = 0;
  uint8_t local_bleCount = 0;

  // --- Iterate and Populate JSON ---
  for (auto const& grp : sortedGroups) {
    const std::vector<const DeviceInfo*>& devices = grp.second;
    if (devices.empty()) continue;

    int strongestRSSI = INT_MIN;
    const char* name = "";
    const char* type_str = "";
    std::string primary_mac_address;

    // Aggregate properties from all devices in the correlated group
    for (auto const& dev : devices) {
      if (dev->rssi > strongestRSSI) {
        strongestRSSI = dev->rssi;
        name = dev->name;
        type_str = dev->type;
      }
      
      // Get the MAC for the 'Address' field (any MAC for the group will do, 
      // but the first one found is used here).
      for (auto const& [mac_num, status_ptr] : dev->macs) {
          if (primary_mac_address.empty()) {
             primary_mac_address = uint64ToMacString(mac_num);
             break; // Use the first MAC found
          }
      }
    }

    bool is_wifi_ap = (strcmp(type_str, "Wi-Fi AP") == 0);
    bool is_wifi_client = (strcmp(type_str, "Wi-Fi Client") == 0);
    bool is_ble = (strcmp(type_str, "BLE") == 0);
    bool is_wifi = is_wifi_ap || is_wifi_client;

    if ((is_wifi && local_wifiCount >= maxWifi) || (is_ble && local_bleCount >= maxBle)) {
      continue;
    }

    // --- JSON Object Population ---
    
    if (is_wifi) {
      if (local_wifiCount < maxWifi) {
        JsonObject obj = wifiArray.createNestedObject();
        
        // Wi-Fi objects use "SSID" (Name) and "RSSI"
        obj["SSID"] = name;
        obj["RSSI"] = strongestRSSI;
        
        local_wifiCount++;
      }
    } else if (is_ble) {
      if (local_bleCount < maxBle) {
        JsonObject obj = bleArray.createNestedObject();
        
        // BLE objects use "Name", "Address", and "RSSI"
        obj["Name"] = name;
        obj["Address"] = primary_mac_address;
        obj["RSSI"] = strongestRSSI;
        
        local_bleCount++;
      }
    }
  }
  
  // --- Final Serialization ---
  std::string dataPayload;
  dataPayload.reserve(MQTT_BUFFER_SIZE); 
  
  if (serializeJson(doc, dataPayload) == 0) {
    if (DEBUG && Serial) Serial.println("[JSON FAILED] Failed to serialize JSON.");
    return "";
  }
  
  return dataPayload;
}

#endif  // DEVICE_SCANNER_V4_H