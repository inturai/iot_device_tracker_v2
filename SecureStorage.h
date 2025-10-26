#ifndef SECURE_STORAGE_H
#define SECURE_STORAGE_H

#include <Arduino.h>
#include <AESLib.h>
#include <Preferences.h>
#include <Base64.h>
#include <MD5Builder.h>
#include <string>     // REQUIRED for std::string
#include <sstream>    // REQUIRED for number to std::string conversion

class SecureStorage {
private:
  AESLib aesLib;
  Preferences preferences;
  std::string storageNamespace; // REPLACED String
  bool readOnly;

  byte aesKey[16];  // AES key derived from device ID
  const byte iv[16] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
  };

  // 🔐 Generate AES key from ESP32's unique MAC address
  void generateKeyFromDeviceID() {
    uint64_t chipId = ESP.getEfuseMac();
    
    // Convert 64-bit ID to two 32-bit hex strings and concatenate using std::string
    std::ostringstream ss;
    ss << std::hex << (uint32_t)(chipId >> 32) << (uint32_t)chipId;
    std::string idStr = ss.str();

    MD5Builder md5;
    md5.begin();
    md5.add(idStr.c_str()); // Use .c_str() for MD5
    md5.calculate();
    md5.getBytes(aesKey);
  }

  // 📦 Apply PKCS#7 padding
  std::string applyPadding(std::string input) { // REPLACED String
    int padLen = 16 - (input.length() % 16);
    for (int i = 0; i < padLen; i++) {
      input += (char)padLen;
    }
    return input;
  }

  // 🔐 Encrypt and encode as Base64
  std::string encryptAES(std::string input) { // REPLACED String
    input = applyPadding(input);
    
    // Copy content to C-style array for AESLib, size is length() + 1 for safety but we only care about length()
    size_t inputLen = input.length();
    byte plainText[inputLen];
    memcpy(plainText, input.c_str(), inputLen);
    
    // CipherText buffer size must be >= inputLen + 16 (for 128-bit key block size and padding)
    byte cipherText[256] = { 0 }; 
    byte ivCopy[16];
    memcpy(ivCopy, iv, sizeof(iv));

    // AESLib encrypts a length, not a null-terminated string
    aesLib.encrypt(plainText, inputLen, cipherText, aesKey, 128, ivCopy);

    // Base64 encoding requires a char* output buffer
    // The output size of Base64 is roughly (4/3) * input size, plus padding/null
    char base64Encoded[512]; 
    base64_encode(base64Encoded, (char*)cipherText, inputLen); // inputLen is the length of cipherText here

    return std::string(base64Encoded); // Convert back to std::string
  }

  // 🔓 Decode Base64 and decrypt
  std::string decryptAES(std::string encryptedBase64) { // REPLACED String
    byte encryptedData[256] = { 0 };
    // Use .c_str() for Base64 decode
    int encryptedLength = base64_decode((char*)encryptedData, encryptedBase64.c_str(), encryptedBase64.length());

    byte decryptedText[256] = { 0 };
    byte ivCopy[16];
    memcpy(ivCopy, iv, sizeof(iv));

    // AESLib decrypts a length
    int decryptedLength = aesLib.decrypt(encryptedData, encryptedLength, decryptedText, aesKey, 128, ivCopy);
    
    if (decryptedLength <= 0) {
      Serial.println("AES Decryption failed!");
      return "";
    }

    // PKCS#7 unpadding logic
    int padLen = decryptedText[decryptedLength - 1];
    
    if (padLen > 0 && padLen <= 16 && (decryptedLength - padLen) >= 0) {
      // Correctly terminate the string before the padding bytes
      decryptedText[decryptedLength - padLen] = '\0';
    } else {
      Serial.println("Padding error! Check decrypted output. Returning raw string.");
      decryptedText[decryptedLength] = '\0'; // Ensure termination even if padding is wrong
    }

    return std::string((char*)decryptedText); // Convert C-string to std::string
  }

public:
  // 🏗️ Constructor with optional namespace and read-only flag
  // Keep const char* for compatibility with C-style usage
  SecureStorage(const char* ns = "secure_storage", bool ro = false) {
    storageNamespace = ns; // std::string assignment from const char*
    readOnly = ro;
  }

  // 📂 Begin with default namespace
  void begin(bool readOnlyMode = false) {
    readOnly = readOnlyMode;
    generateKeyFromDeviceID();
    // Use .c_str() for Preferences
    preferences.begin(storageNamespace.c_str(), readOnly); 
  }

  // 📂 ✅ Begin with custom namespace and read-only flag (like Preferences)
  void begin(const char* ns, bool readOnlyMode = false) {
    storageNamespace = ns; // std::string assignment from const char*
    readOnly = readOnlyMode;
    generateKeyFromDeviceID();
    preferences.begin(ns, readOnlyMode);
  }

  // ❌ End Preferences session
  void end() {
    preferences.end();
  }

  // 💾 Store encrypted string (REPLACED String with std::string)
  void putEncryptedString(const char* key, const std::string& value) {
    std::string encrypted = encryptAES(value);
    // Use .c_str() for Preferences
    preferences.putString(key, encrypted.c_str()); 
  }

  // 📥 Retrieve and decrypt string (REPLACED String with std::string)
  std::string getDecryptedString(const char* key) {
    // Preferences returns Arduino String, convert it to std::string immediately
    std::string encrypted = preferences.getString(key, "").c_str(); 
    
    if (!encrypted.empty()) { // Use .empty()
      return decryptAES(encrypted);
    } else {
      return "";
    }
  }

  // 💾 Store plain string (REPLACED String with std::string)
  void putString(const char* key, const std::string& value) {
    // Use .c_str() for Preferences
    preferences.putString(key, value.c_str()); 
  }

  // 📥 Retrieve plain string (REPLACED String with std::string)
  std::string getString(const char* key, const std::string& defaultValue = "") {
    // Preferences returns Arduino String, convert it to std::string immediately
    return preferences.getString(key, defaultValue.c_str()).c_str(); 
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