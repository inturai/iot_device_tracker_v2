#ifndef SECURE_STORAGE_H
#define SECURE_STORAGE_H

#include <Arduino.h>
#include <AESLib.h>
#undef dump
#include <Preferences.h>
#include <Base64.h>
#include <MD5Builder.h>

class SecureStorage {
private:
  AESLib aesLib;
  Preferences preferences;
  String storageNamespace;
  bool readOnly;

  byte aesKey[16];  // AES key derived from device ID
  const byte iv[16] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
  };

  // 🔐 Generate AES key from ESP32's unique MAC address
  void generateKeyFromDeviceID() {
    uint64_t chipId = ESP.getEfuseMac();
    String idStr = String((uint32_t)(chipId >> 32), HEX) + String((uint32_t)chipId, HEX);

    MD5Builder md5;
    md5.begin();
    md5.add(idStr);
    md5.calculate();
    md5.getBytes(aesKey);
  }

  // 📦 Apply PKCS#7 padding
  String applyPadding(String input) {
    int padLen = 16 - (input.length() % 16);
    for (int i = 0; i < padLen; i++) {
      input += (char)padLen;
    }
    return input;
  }

  // 🔐 Encrypt and encode as Base64
  String encryptAES(String input) {
    input = applyPadding(input);
    byte plainText[input.length()];
    input.getBytes(plainText, input.length() + 1);

    byte cipherText[128] = { 0 };
    byte ivCopy[16];
    memcpy(ivCopy, iv, sizeof(iv));

    aesLib.encrypt(plainText, input.length(), cipherText, aesKey, 128, ivCopy);

    char base64Encoded[256];
    base64_encode(base64Encoded, (char*)cipherText, input.length());

    return String(base64Encoded);
  }

  // 🔓 Decode Base64 and decrypt
  String decryptAES(String encryptedBase64) {
    byte encryptedData[128] = { 0 };
    int encryptedLength = base64_decode((char*)encryptedData, encryptedBase64.c_str(), encryptedBase64.length());

    byte decryptedText[128] = { 0 };
    byte ivCopy[16];
    memcpy(ivCopy, iv, sizeof(iv));

    int decryptedLength = aesLib.decrypt(encryptedData, encryptedLength, decryptedText, aesKey, 128, ivCopy);
    if (decryptedLength <= 0) {
      Serial.println("AES Decryption failed!");
      return "";
    }

    int padLen = decryptedText[decryptedLength - 1];
    if (padLen > 0 && padLen <= 16) {
      decryptedText[decryptedLength - padLen] = '\0';
    } else {
      Serial.println("Padding error! Check decrypted output.");
    }

    return String((char*)decryptedText);
  }

public:
  // 🏗️ Constructor with optional namespace and read-only flag
  SecureStorage(const char* ns = "secure_storage", bool ro = false) {
    storageNamespace = String(ns);
    readOnly = ro;
  }

  // 📂 Begin with default namespace
  void begin(bool readOnlyMode = false) {
    readOnly = readOnlyMode;
    generateKeyFromDeviceID();
    preferences.begin(storageNamespace.c_str(), readOnly);
  }

  // 📂 ✅ Begin with custom namespace and read-only flag (like Preferences)
  void begin(const char* ns, bool readOnlyMode = false) {
    storageNamespace = String(ns);
    readOnly = readOnlyMode;
    generateKeyFromDeviceID();
    preferences.begin(ns, readOnlyMode);
  }

  // ❌ End Preferences session
  void end() {
    preferences.end();
  }

  // 💾 Store encrypted string
  void putEncryptedString(const char* key, const String& value) {
    String encrypted = encryptAES(value);
    preferences.putString(key, encrypted);
  }

  // 📥 Retrieve and decrypt string
  String getDecryptedString(const char* key) {
    String encrypted = preferences.getString(key, "");
    return decryptAES(encrypted);
  }

  // 💾 Store plain string
  void putString(const char* key, const String& value) {
    preferences.putString(key, value);
  }

  // 📥 Retrieve plain string
  String getString(const char* key, const String& defaultValue = "") {
    return preferences.getString(key, defaultValue);
  }

  // 💾 Store integer
  void putInt(const char* key, int value) {
    preferences.putInt(key, value);
  }

  // 📥 Retrieve integer
  int getInt(const char* key, int defaultValue = 0) {
    return preferences.getInt(key, defaultValue);
  }

  // 💾 Store boolean
  void putBool(const char* key, bool value) {
    preferences.putBool(key, value);
  }

  // 📥 Retrieve boolean
  bool getBool(const char* key, bool defaultValue = false) {
    return preferences.getBool(key, defaultValue);
  }

  // 🗑️ Remove key
  void remove(const char* key) {
    preferences.remove(key);
  }

  // 🧹 Clear all keys
  void clear() {
    preferences.clear();
  }
};

#endif  // SECURE_STORAGE_H
