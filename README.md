
# 🌐 Triangulation Access Point V2 / BLE & WiFi Gateway V2

This project utilizes an **ESP32** to function as a resilient and secure gateway for **device location and asset tracking**. It continuously scans the surrounding environment for both **Bluetooth Low Energy (BLE)** devices, **Wi-Fi devices** and **Wi-Fi networks**, aggregates the data, and reports it to an external server via **MQTT**.

-----

## ✨ Core Functionality

  * **Dual-Band Scanning:** Performs simultaneous scanning for both **BLE advertising packets** and nearby **Wi-Fi Access Points**.
  * **Data Filtering & Sorting:** Filters scan results based on a configured **RSSI threshold** (signal strength) and sorts devices to prioritize the strongest signals for the best triangulation or proximity analysis.
  * **Secure Configuration:** Uses **Encrypted Non-Volatile Storage (`SecureStorage`)** on the ESP32 to securely persist sensitive information, including Wi-Fi credentials and MQTT broker passwords.
  * **Resilient Connectivity:** Implements **exponential backoff retry logic** to automatically re-establish connections to both the **Wi-Fi network** and the **MQTT broker**, ensuring high uptime.
  * **Secure MQTT Communication:** Pre-configured with a **CA Certificate** to support secure **MQTT over TLS/SSL**.
  * **Structured Reporting:** Publishes data as **JSON payloads** to distinct MQTT topics:
      * **Data Topic:** Comprehensive list of scanned BLE and Wi-Fi devices with their MAC addresses and RSSI values.
      * **Health Topic:** Regular status updates, including device uptime.
  * **Serial Management:** Allows for runtime configuration and maintenance via the Serial interface, including commands to **reload** or **clear all stored configuration**.

-----

## 📦 MQTT Payload Descriptions

The device publishes structured data to the MQTT broker using the following JSON formats:

### 1\. Device Scan Data (Topic: `TOPIC_DATA`)

This payload provides a snapshot of all filtered Wi-Fi and BLE devices detected during a scan cycle.

| Field | Type | Description |
| :--- | :--- | :--- |
| `device_id` | `string` | Unique identifier of the ESP32 Gateway. |
| `wifi` | `array` | List of detected Wi-Fi networks (SSIDs and RSSI). |
| `wifi[].SSID` | `string` | Wi-Fi network name. |
| `wifi[].RSSI` | `integer` | Signal strength in dBm. |
| `ble` | `array` | List of detected BLE devices (Name, Address, RSSI). |
| `ble[].Name` | `string` | Advertised local name of the BLE device. |
| `ble[].Address` | `string` | MAC address of the BLE device. |
| `ble[].RSSI` | `integer` | Signal strength in dBm. |

```json
{
  "device_id": "YOUR_DEVICE_MAC_ADDRESS",
  "wifi": [
    { "SSID": "Home-AP", "RSSI": -55 }
  ],
  "ble": [
    { 
      "Name": "Asset_Tag_1", 
      "Address": "aa:bb:cc:dd:ee:ff", 
      "RSSI": -70 
    }
  ]
}
```

### 2\. System Health Data (Topic: `TOPIC_HEALTH`)

Published periodically (every 30 seconds) to provide operational status.

| Field | Type | Description |
| :--- | :--- | :--- |
| `status` | `string` | Current operational state (always `"active"`). |
| `uptime_seconds` | `integer` | Time elapsed since the last reboot, in seconds. |

```json
{
  "status": "active",
  "uptime_seconds": 12345
}
```

-----

## 🛠️ Configuration & Dependencies

| Component | Detail |
| :--- | :--- |
| **Hardware** | ESP32 (requires BLE support) |
| **Libraries** | `WiFi`, `BLEDevice`, `PubSubClient`, `ArduinoJson`, `SecureStorage` |
| **Key Configs** | Wi-Fi SSID/Password, MQTT Server/Credentials, RSSI Filtering Thresholds. |

  * **Setup Note:** All network and broker configuration is loaded from the ESP32's secure storage and must be initially set via a separate configuration mode or serial command.