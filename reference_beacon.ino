/* ===========================================================================
   reference_beacon.ino
   ---------------------------------------------------------------------------
   Calibration transmitter for Vulpecula. Flash to ANY spare ESP32
   (classic, C3, S3, C6 - it does not need to be a C5).

   Why you need this rather than just using your phone: calibration is only
   meaningful against a source whose transmit power is known and constant. A
   phone changes power dynamically, and you cannot write its EIRP on the case.

   It advertises on BLE with an explicit TX Power Level field (AD type 0x0A),
   which also lets you exercise the reference-based BLE gating path rather
   than only the raw-RSSI fallback. It also raises a WiFi SoftAP at a pinned
   transmit power so the 2.4 GHz pass has something to measure.

   ---------------------------------------------------------------------------
   THE NAME IS DELIBERATELY ALARMING.

   A calibration transmitter that could be mistaken for a real detection
   during a later sweep is a hazard. "DO-NOT-TRUST" in the name means it can
   never quietly end up in a report as a finding.
   ---------------------------------------------------------------------------

   Procedure is in CALIBRATION.md. Short version: tape-measure 2.00 m, let the
   detector collect 40+ samples, move to 0.30 m, collect again, commit.

   Write the configured power on the enclosure in marker. The whole value of a
   reference is that you know what it transmits.
   =========================================================================== */

#include <WiFi.h>
#include <esp_wifi.h>
#include <NimBLEDevice.h>

#define REF_NAME     "VULPECULA-CAL-REF-DO-NOT-TRUST"
#define REF_AP_PASS  "calibration"

// BLE transmit power. ESP_PWR_LVL_P3 is roughly +3 dBm on most ESP32 parts.
// Whatever you pick, keep it FIXED across sessions or your thresholds move.
#define REF_BLE_PWR  ESP_PWR_LVL_P3

// WiFi transmit power in 0.25 dBm units. 44 = 11 dBm. Pinned rather than left
// at the default so the reference is reproducible.
#define REF_WIFI_PWR_QDBM 44

void setup()
{
    Serial.begin(115200);
    delay(300);

    // --- BLE: legacy advert carrying an explicit TX Power Level field ---
    NimBLEDevice::init(REF_NAME);
    NimBLEDevice::setPower(REF_BLE_PWR);

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    NimBLEAdvertisementData data;
    data.setName(REF_NAME);
    data.addTxPower();                       // AD type 0x0A - the point
    adv->setAdvertisementData(data);
    // 100 ms interval: fast, so the detector reaches 40 samples quickly.
    adv->setMinInterval(160);                // 160 * 0.625 ms
    adv->setMaxInterval(160);
    adv->start();

    // --- WiFi: a SoftAP so the 2.4 GHz pass has a target ---
    WiFi.mode(WIFI_AP);
    WiFi.softAP(REF_NAME, REF_AP_PASS);
    esp_wifi_set_max_tx_power(REF_WIFI_PWR_QDBM);

    int8_t got = 0;
    esp_wifi_get_max_tx_power(&got);
    Serial.printf("reference beacon up\n");
    Serial.printf("  name      : %s\n", REF_NAME);
    Serial.printf("  WiFi power: %.2f dBm (requested %.2f)\n",
                  got * 0.25f, REF_WIFI_PWR_QDBM * 0.25f);
    Serial.printf("  BLE advert: 100 ms, TX Power Level field present\n");
    Serial.printf("\nWrite the WiFi power on the enclosure.\n");
}

void loop()
{
    // Nothing to do: both radios beacon on their own. Kept awake rather than
    // sleeping, because a reference that intermittently vanishes makes
    // calibration samples meaningless.
    delay(5000);
}
