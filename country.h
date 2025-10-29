#pragma once
// wifi_country_utils.h
// Utility to set Wi-Fi country configuration based on country code.
// Safe to include once across all includes.

#include "esp_wifi.h"
#include "WiFi.h"
#include <string.h>

// Optional: Define DEBUG if not already defined elsewhere
#ifndef DEBUG
#define DEBUG 0
#endif

/**
 * @brief Sets the Wi-Fi country configuration according to the provided country code.
 *
 * Supported zones:
 *   - Zone 1: Channels 1–11 (e.g., US, CA, MX, BR, SG)
 *   - Zone 2: Channels 1–13 (e.g., EU, IN, AU, CN, GB, etc.)
 *   - Zone 3: Channels 1–14 (JP)
 *
 * Unsupported codes default to Zone 2.
 *
 * @param countryCode Two-letter ISO country code (e.g., "US", "JP", "EU")
 */
inline void setWifiCountryByString(const char* countryCode) {
  wifi_country_t country = {0};

  // Copy 2 characters safely and null-terminate
  strncpy((char*)country.cc, countryCode, 2);
  country.cc[2] = '\0';
  country.policy = WIFI_COUNTRY_POLICY_AUTO;

  // --- Determine channel limits ---
  if (
    strcmp(countryCode, "US") == 0 || strcmp(countryCode, "CA") == 0 ||
    strcmp(countryCode, "MX") == 0 || strcmp(countryCode, "BR") == 0 ||
    strcmp(countryCode, "SG") == 0
  ) {
    country.schan = 1;
    country.nchan = 11;
  }
  else if (strcmp(countryCode, "JP") == 0) {
    country.schan = 1;
    country.nchan = 14;
  }
  else if (
    strcmp(countryCode, "EU") == 0 || strcmp(countryCode, "IN") == 0 ||
    strcmp(countryCode, "AU") == 0 || strcmp(countryCode, "NZ") == 0 ||
    strcmp(countryCode, "CN") == 0 || strcmp(countryCode, "HK") == 0 ||
    strcmp(countryCode, "GB") == 0 || strcmp(countryCode, "DE") == 0 ||
    strcmp(countryCode, "FR") == 0 || strcmp(countryCode, "IT") == 0 ||
    strcmp(countryCode, "ES") == 0 || strcmp(countryCode, "TW") == 0 ||
    strcmp(countryCode, "AT") == 0 || strcmp(countryCode, "BE") == 0 ||
    strcmp(countryCode, "CH") == 0 || strcmp(countryCode, "DK") == 0 ||
    strcmp(countryCode, "GR") == 0 || strcmp(countryCode, "IE") == 0 ||
    strcmp(countryCode, "IS") == 0 || strcmp(countryCode, "NL") == 0 ||
    strcmp(countryCode, "NO") == 0 || strcmp(countryCode, "PT") == 0 ||
    strcmp(countryCode, "SE") == 0
  ) {
    country.schan = 1;
    country.nchan = 13;
  }
  else {
    if (DEBUG)
      Serial.printf("Unsupported country code %s. Defaulting to Zone 2 (Channels 1–13).\n", countryCode);
    strcpy((char*)country.cc, "EU");
    country.schan = 1;
    country.nchan = 13;
  }

  // --- Apply the setting ---
  esp_err_t err = esp_wifi_set_country(&country);

  // --- Report result ---
  if (DEBUG) {
    if (err != ESP_OK) {
      Serial.printf("Failed to set Wi-Fi country. Error: 0x%X\n", err);
    } else {
      Serial.printf("Wi-Fi country set to %s (Channels: %d–%d)\n",
                    country.cc, country.schan, country.schan + country.nchan - 1);
    }
  }
}
