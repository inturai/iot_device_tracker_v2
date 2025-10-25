# 🌐 Triangulation Access Point / BLE & WiFi Gateway

This project utilizes an **ESP32** to function as a resilient and secure gateway for **device location and asset tracking**. It continuously scans the surrounding environment for both **Bluetooth Low Energy (BLE)** devices, **Wi-Fi devices** and **Wi-Fi networks**, aggregates the data, and reports it to an external server via **MQTT**.

---

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

---

## 🛠️ Configuration & Dependencies

| Component | Detail |
| :--- | :--- |
| **Hardware** | ESP32 (requires BLE support) |
| **Libraries** | `WiFi`, `BLEDevice`, `PubSubClient`, `ArduinoJson`, `SecureStorage` |
| **Key Configs** | Wi-Fi SSID/Password, MQTT Server/Credentials, RSSI Filtering Thresholds. |

* **Setup Note:** All network and broker configuration is loaded from the ESP32's secure storage and must be initially set via a separate configuration mode or serial command.