/* ===========================================================================
   Vulpecula.ino
   ---------------------------------------------------------------------------
   Proximity-gated dual-band WiFi + BLE recording-device detector
   for the RockBase IoT NM-CYD-C5  (ESP32-C5-WROOM-1, 2.8" 320x240 touch).

   Single-file Arduino sketch. Receive only: no probe requests, no active BLE
   scanning, no association, nothing transmitted.

   ---------------------------------------------------------------------------
   WHAT THIS IS FOR

   Sweeping a space you occupy or are authorised to assess - a hotel room, a
   rental, an office - for wireless cameras and microphones.

   The ~2 m gate is a SPATIAL FILTER, not a sensitivity limit. Room coverage
   comes from you walking the board around. At full sensitivity a hotel gives
   you 100-300 BLE devices and 50+ access points; the gate cuts that to single
   digits. That is the difference between an instrument and a log file.

   ---------------------------------------------------------------------------
   ARDUINO IDE SETUP   (IDE 2.x)

   1. Boards Manager -> "esp32" by Espressif, version 3.3.5 or newer.
      Older cores have no ESP32-C5 support and will not compile.

   2. Library Manager -> "NimBLE-Arduino" by h2zero, version 2.2.x or newer.
      That is the ONLY library you need. The display and touch drivers are in
      this file, so there is no TFT_eSPI to install, configure or patch.

   3. Tools menu:
        Board .................. ESP32C5 Dev Module
        USB CDC On Boot ........ Enabled
        Flash Size ............. 16MB (128Mb)
        Partition Scheme ....... Huge APP (3MB No OTA/1MB SPIFFS)
        PSRAM .................. Enabled          <-- required, table lives there
        Upload Speed ........... 460800
        Core Debug Level ....... None

   4. Plug into the ESP32-C5 USB-C port (the one that is NOT the CH340),
      select the port, Upload.

   5. First boot runs a display self-test: colour bars labelled R / G / B.
      If red and blue are swapped, set PANEL_BGR to 0 below and reflash.
      If the screen stays dark, see "if the display does not work".

   ---------------------------------------------------------------------------
   WHY ARDUINO IDE IS FINE HERE, AND WHEN IT IS NOT

   Normally PlatformIO wins for this board, for one reason: TFT_eSPI needs its
   configuration passed as compile-time flags and needs the vendor's
   TFT_eSPI_ESP32_C5.c/h files dropped into the library, because upstream
   TFT_eSPI has no C5 backend. In Arduino IDE that means hand-editing a
   managed library, which gets wiped on every library update.

   This file sidesteps that entirely by talking to the ST7789 over the stock
   Arduino SPI library in about 200 lines. No library patching, so Arduino IDE
   becomes the simpler choice.

   Use PlatformIO instead if you want pinned dependency versions, a custom
   partition table in the repo, or to split this back into modules.

   ---------------------------------------------------------------------------
   PIN PROVENANCE  -  read this before changing anything

   SCK/MISO/MOSI and the three CS pins are from the vendor's own pinout table
   (README and wiki). Backlight and RGB LED likewise.

   TFT_DC = 24 is NOT in the vendor pinout table. It comes from RockBase-iot
   /NM-CYD-C5 issue #3, filed by someone who brought the board up on
   TFT_eSPI and published a working config. Same source for TFT_RST = -1
   (no reset line; we issue a software reset instead) and for SPI at 20 MHz
   rather than 40.

   That issue also documents two colour traps that this file handles:
     - the panel needs INVERSION OFF. TFT_eSPI's ST7789 init table sends
       INVON unconditionally, which is why people see red render as yellow
       or cyan. We send INVOFF (0x20) instead. See PANEL_INVERT.
     - colour order may need to be BGR rather than RGB. See PANEL_BGR.

   The wiki calls the touch panel capacitive while also listing an XPT2046 on
   GPIO 1. XPT2046 is a resistive controller, so the wiki contradicts itself.
   This file drives it as resistive SPI, which matches the vendor README and
   every community project. If your unit is genuinely capacitive, touch will
   not respond and you will need an I2C driver on IO8/IO9 instead - the rest
   of the firmware still works, it is just read-only.

   ---------------------------------------------------------------------------
   IF THE DISPLAY DOES NOT WORK

   Set SERIAL_ONLY to 1. The whole detector runs headless over the serial
   monitor at 115200, so you can verify the radio side before fighting the
   panel. Then try, in order:
     - PANEL_DRIVER 1 (ILI9341). The vendor says some units ship that way.
     - PANEL_BGR 0
     - PANEL_INVERT 1
     - SPI_HZ_LCD 10000000
     - TFT_DC 21, then 22, then 20. Issue #3 is one report, not a datasheet.

   ---------------------------------------------------------------------------
   CALIBRATE BEFORE YOU TRUST A READING

   The thresholds below are path-loss maths plus an assumed implementation
   margin. They are a starting point, not a measurement of YOUR board and
   antenna. SETUP screen -> right half starts the guided routine. Full
   procedure is in CALIBRATION.md alongside this sketch.

   The one measurement that matters most: put a reference transmitter 2 m away
   on the far side of an interior wall and confirm it reads ambient. Rejecting
   the next room is the entire premise.

   ---------------------------------------------------------------------------
   LIMITS  -  a tool that implies "all clear" is worse than no tool

   - A camera recording to SD with its radio off is INVISIBLE here. Largest
     blind spot, unfixable in firmware. Always also sweep for lenses.
   - LTE/4G cameras, wired cameras and analog transmitters are invisible.
   - No detector can tell whether a device is recording.
   - Absence of an alert is never proof of absence.

   Licence: MIT. Signature tables are leads requiring field confirmation,
   not proof of anything. Radio, privacy and surveillance law varies; none of
   this is legal advice.
   =========================================================================== */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <esp_random.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <mbedtls/sha256.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <NimBLEDevice.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

// ===========================================================================
// BOARD  -  see PIN PROVENANCE above before editing
// ===========================================================================
#define PIN_SCK        6
#define PIN_MISO       2
#define PIN_MOSI       7
#define PIN_LCD_CS    23
#define PIN_LCD_DC    24     // issue #3, not the vendor pinout table
#define PIN_LCD_RST   -1     // no reset line; software reset instead
#define PIN_TOUCH_CS   1
#define PIN_SD_CS     10
#define PIN_BL        25
#define PIN_WS2812    27

#define SPI_HZ_LCD    20000000
#define SPI_HZ_TOUCH   2500000
#define SPI_HZ_SD     20000000

#define PANEL_DRIVER  0       // 0 = ST7789 (default), 1 = ILI9341
#define PANEL_BGR     1       // 1 = BGR colour order, 0 = RGB
#define PANEL_INVERT  0       // 0 = send INVOFF (this panel), 1 = INVON
#define PANEL_ROT     3       // 3 = landscape, USB to the left

#define SCR_W       320
#define SCR_H       240

#define SERIAL_ONLY   0       // 1 = headless, run without the display

// Draw all on-screen text in upper case.
//
// The 6x8 cell has no room below the baseline, so g j p q y are clipped and
// read as o i b a v. Rather than shrink the cap height to buy two descender
// rows - which would make everything harder to read at arm's length - the
// display is upper case throughout.
//
// This applies to captured strings too: SSIDs, vendor and model names. Those
// are case-sensitive evidence, so the SD LOG STILL RECORDS THEM EXACTLY AS
// RECEIVED - only the screen is folded. If you need an SSID's true case, read
// it from the CSV.
#define UI_ALL_CAPS   1

// Resistive touch calibration. Defaults are typical for this panel; the
// SETUP screen prints raw values so you can correct them.
#define TOUCH_RAW_X_MIN  300
#define TOUCH_RAW_X_MAX 3800
#define TOUCH_RAW_Y_MIN  300
#define TOUCH_RAW_Y_MAX 3800
#define TOUCH_Z_THRESH   350
#define TOUCH_SWAP_XY      1
#define TOUCH_INVERT_X     0
#define TOUCH_INVERT_Y     1

// ===========================================================================
// PROXIMITY GATE  -  the design centre
//
// Three tiers:
//   CONTACT  ~0.3 m  confirmation. Open the fixture.
//   NEAR     ~2.0 m  alert. Worth investigating.
//   ambient  beyond  logged silently, never alerts, but KEPT - the corridor
//                    vs room baseline diff depends on sub-threshold history.
//
// Budget at 2.4 GHz for a +17 dBm camera against the -35 dBm NEAR gate:
//   in room 2 m        17 - 46            = -29  pass
//   in room 4 m        17 - 52            = -35  borderline
//   next room 2 m      17 - 46 - 12(wall) = -41  reject
//   corridor 8 m       17 - 58 -  6(door) = -47  reject
// Interior hotel demise walls run 10-20 dB at 2.4 GHz, so distance gives
// ~6 dB of separation and the wall stacks 10-20 dB on top of that.
// ===========================================================================
#define THRESH_NEAR_24     (-35)
// CONTACT was -20 in an earlier revision. A +20 dBm camera 0.3 m behind a
// 10 dB wall reads -19.7 dBm and would have tripped the confirmation tier
// from the next room, so it is -15. Cost: in-room devices below about
// +13 dBm now only reach CONTACT at ~0.2 m rather than 0.3 m. Worth it for a
// tier whose whole value is that you can trust it.
#define THRESH_CONTACT_24  (-15)
#define THRESH_NEAR_5      (-42)   // 5 GHz loses ~7 dB more at the same range
#define THRESH_CONTACT_5   (-22)
// BLE raw fallback, used ONLY when an advert carries no transmit-power
// reference. No wall-proof claim is made for these: consumer BLE TX power
// spans about -20..+8 dBm, so a +8 dBm advertiser through a wall and a
// -4 dBm advertiser in the room genuinely overlap. The reference-based
// distance path below is the real answer for BLE.
#define THRESH_NEAR_BLE    (-55)
#define THRESH_CONTACT_BLE (-40)

#define TIER_HYSTERESIS_DB   4     // or the display flickers and reads broken
#define MIN_PKTS_FOR_NEAR    3     // one lucky reflection must not alert
#define RSSI_RING_LEN       32
#define PEAK_WINDOW_MS    2000
#define MEDIAN_WINDOW_MS  4000

// --- BLE ranging -----------------------------------------------------------
// Where an advert carries TX Power Level (AD 0x0A), an iBeacon measured
// power, or Eddystone ranging data, we compute path loss directly and remove
// transmit power as an error term.
#define BLE_1M_LOSS_DB      47     // radiated power -> RSSI at 1 m. Calibratable.
#define PATHLOSS_EXPONENT  2.0f    // n=2 free space; stays near 2 inside 2 m
#define DIST_NEAR_M        2.0f
#define DIST_CONTACT_M     0.30f

// ===========================================================================
// SCHEDULER
//
// Three hard constraints on this chip:
//   1. no simultaneous dual-band - esp_wifi_set_band_mode picks one
//   2. one antenna on WROOM-1, software time-division, no GPIO switching
//   3. WiFi and BLE also share time by coexistence
//
// So detection probability at a sweep position is radio-on dwell versus the
// target's transmit interval. BLE adverts range 20 ms to 10.24 s. A three-way
// round-robin at 33% duty needs 30+ s of wall clock per position to cover a
// 10 s advertiser - unusable while walking. Hence SEPARATE PASSES: two fast
// walks instead of one slow one. Round-robin is reserved for stationary WATCH.
// ===========================================================================
#define WIFI_BAND_SLICE_MS    1500
#define WIFI_CH_DWELL_MS       120   // beacons are ~100 ms
#define WATCH_SLICE_MS        4000
#define BAND_SWITCH_SETTLE_MS   30

static const uint8_t CH_24[] = {1, 6, 11};
// 5 GHz candidates. Which are actually settable is PROBED AT BOOT: the C5
// supports DFS channels but only passive radar detection, so the regulatory
// table may refuse 52-144 depending on country config. We measure instead of
// assuming, and report the answer on the SETUP screen.
static const uint8_t CH_5_CANDIDATES[] = {
    36, 40, 44, 48,
    52, 56, 60, 64,
    100,104,108,112,116,120,124,128,132,136,140,144,
    149,153,157,161,165
};

// Dwell accounting. No IMU on this board, so a new position is either marked
// by tapping the screen or inferred when the strongest peak moves.
#define AUTO_REMARK_DELTA_DB    6
#define SWEEP_WARN_BELOW_MS  1200

// ===========================================================================
// TRACK TABLE  (PSRAM - we have 8 MB, so nothing gets thrown away)
// ===========================================================================
#define MAX_TRACKS           600
#define NAME_LEN              32
#define SWEEP_PROFILE_SLOTS   24

// Streaming heuristic. A device inside the gate that is also pushing
// sustained uplink-dominant traffic is close to conclusive, and needs no
// vendor database at all - which is why it survives MAC randomisation.
#define STREAM_MIN_PKT_RATE       30
#define STREAM_MIN_UPLINK_RATIO  4.0f
#define STREAM_MIN_BYTES_TX   200000
#define STREAM_WINDOW_MS        3000

#define BASELINE_CAPTURE_MS    60000
#define ROOM_ONLY_DELTA_DB        12

#define SCORE_POSSIBLE  30
#define SCORE_LIKELY    60
#define SCORE_HIGH      85

#define LOG_DIR "/vulpecula"

// NVS namespace and keys. Declared here rather than beside the proximity
// code because the touch calibration, which appears earlier in the file,
// also persists to it.
static const char *NVS_NS  = "vulpecula";
static const char *NVS_KEY = "prox_cal";

// ===========================================================================
// TYPE DEFINITIONS
//
// DO NOT MOVE THESE BACK DOWN NEXT TO THE CODE THAT USES THEM.
//
// The Arduino IDE preprocessor generates a forward declaration for every
// function in a .ino and injects the whole set at the FIRST function
// definition it finds. Any injected prototype that mentions a type declared
// later in the file then fails to parse, and the errors cascade into dozens
// of misleading messages like
//
//     error: variable or field 'ring_reset' declared void
//     error: 'tier_name' redeclared as different kind of entity
//
// pointing at code that is perfectly correct. Keeping every struct, enum and
// typedef above the first function definition is what makes a single-file
// .ino of this size compile at all. This is also why plain g++ builds of this
// file succeed even when the IDE fails: g++ does no prototype injection.
// ===========================================================================

typedef enum { BAND_24 = 0, BAND_5, BAND_BLE, BAND_COUNT } rband_t;
typedef enum { TIER_AMBIENT = 0, TIER_NEAR, TIER_CONTACT } tier_t;

typedef struct { int8_t rssi; uint32_t ts; } rssi_sample_t;
typedef struct {
    rssi_sample_t s[RSSI_RING_LEN];
    uint8_t head, count;
} rssi_ring_t;

typedef enum {
    REF_NONE = 0,
    REF_ADV_TXPOWER,   // AD type 0x0A
    REF_IBEACON,       // calibrated RSSI at 1 m - the best case
    REF_EDDYSTONE      // ranging power at 0 m
} ref_source_t;

typedef struct { ref_source_t src; int8_t rssi_at_1m; } ranging_ref_t;

typedef struct {
    int8_t near_thresh[BAND_COUNT];
    int8_t contact_thresh[BAND_COUNT];
    int8_t ble_1m_loss;
    float  pathloss_n;
    bool   calibrated[BAND_COUNT];
} prox_cal_t;

typedef enum { PHASE_FREE = 0, PHASE_CORRIDOR, PHASE_ROOM, PHASE_COUNT } phase_t;

enum {
    E_SOFTAP_CAM_SSID = 1u << 0,
    E_CAM_OUI_STRONG  = 1u << 1,
    E_CAM_OUI_WEAK    = 1u << 2,
    E_STREAM_PROFILE  = 1u << 3,
    E_CONTACT_TIER    = 1u << 4,
    E_ROOM_ONLY       = 1u << 5,
    E_WILDCARD_PROBE  = 1u << 6,
    E_HIDDEN_SSID     = 1u << 7,
    E_BLE_CAM_NAME    = 1u << 8,
    E_BLE_MIC_NAME    = 1u << 9,
    E_BLE_SVC_UUID    = 1u << 10,
    E_BLE_COMPANY     = 1u << 11,
    E_SHARP_PEAK      = 1u << 12,
    E_BROAD_PEAK      = 1u << 13,
    E_WPS_DEVTYPE     = 1u << 14,   // device declared its own type in WPS
    E_BLE_APPEARANCE  = 1u << 15,   // device declared its own BLE appearance
};

// Device type, and how strongly it is known. Tier order is the confidence
// order, so a numerically LOWER tier always wins in dtype_claim().
typedef enum {
    DTYPE_UNKNOWN = 0, DTYPE_CAMERA, DTYPE_MIC, DTYPE_NVR, DTYPE_AV,
    DTYPE_SPEAKER, DTYPE_PHONE, DTYPE_COMPUTER, DTYPE_NETWORK,
    DTYPE_DISPLAY, DTYPE_WEARABLE, DTYPE_PRINTER, DTYPE_TAG
} dtype_t;

#define DT_TIER_A 0   // self-declared: WPS device type, BLE appearance/UUID
#define DT_TIER_B 1   // behaviour: sustained uplink-dominant traffic
#define DT_TIER_C 2   // advertised name or SSID pattern
#define DT_TIER_D 3   // OUI belongs to a camera vendor
#define DT_TIER_NONE 9

// Triage level.
typedef enum { RISK_LOW = 0, RISK_MEDIUM, RISK_HIGH, RISK_CRITICAL } risk_t;

typedef struct {
    bool     used;
    uint8_t  mac[6];
    uint8_t  addr_type;
    rband_t  band;
    uint8_t  channel;

    rssi_ring_t   ring;
    ranging_ref_t ref;
    tier_t   tier, tier_best;
    int8_t   peak_session;
    float    dist_m;

    uint32_t first_ms, last_ms, pkts;

    uint32_t n_beacon, n_probe_req, n_probe_resp, n_mgmt_other;
    uint32_t n_data_up, n_data_down;
    uint32_t bytes_up, bytes_down;
    uint32_t rate_win_start, rate_win_pkts, pkt_rate;
    bool     is_ap;

    char     name[NAME_LEN];
    uint16_t company_id;
    bool     have_company;
    uint16_t svc16[4];
    uint8_t  n_svc16;

    int8_t   peak_phase[PHASE_COUNT];
    uint8_t  phase_mask;

    int8_t   profile[SWEEP_PROFILE_SLOTS];
    uint8_t  profile_head, profile_count;

    uint32_t evidence;
    uint8_t  score;
    bool     alerted;

    // --- identity and type ---
    char     vendor[24];     // from WPS Manufacturer, else the curated OUI table
    char     model[20];      // from WPS Model Name
    uint8_t  vendor_tier;
    uint16_t appearance;     // raw BLE Appearance, 0 if never advertised
    dtype_t  dtype;
    uint8_t  dtype_tier;
    const char *dtype_detail;   // e.g. "security camera", from WPS subcategory

    // --- triage ---
    risk_t   risk, risk_raw;
    uint32_t risk_since;
    const char *risk_why;
} track_t;

typedef enum { SIG_WEAK = 0, SIG_STRONG } sig_weight_t;
typedef struct { const char *pat; sig_weight_t w; } str_sig_t;
typedef struct { uint8_t oui[3]; sig_weight_t w; const char *vendor; } oui_sig_t;

typedef enum { CONF_LOW = 0, CONF_POSSIBLE, CONF_LIKELY, CONF_HIGH } confidence_t;

typedef struct {
    uint8_t  hdr[36];       // enough for a 4-address header plus QoS
    uint16_t len;
    int8_t   rssi;
    uint8_t  channel, band;
} cap_item_t;

typedef struct {
    uint8_t mac[6], addr_type;
    int8_t  rssi;
    uint8_t adv_len, adv[62];
} ble_item_t;

typedef enum { MODE_IDLE = 0, MODE_WIFI_SWEEP, MODE_BLE_SWEEP,
               MODE_SWEEP_ALL, MODE_WATCH } sweep_mode_t;

typedef enum { CAL_IDLE = 0, CAL_NEAR, CAL_CONTACT, CAL_DONE, CAL_FAILED } cal_state_t;

// One curated OUI entry. dtype is a hint recorded at TIER D only.
typedef struct {
    uint8_t      oui[3];
    const char  *vendor;
    dtype_t      dtype;
    sig_weight_t w;
} vendor_sig_t;

// A self-erasing, self-diffing text field on the display. The flicker in the
// first build came from clearing a region and repainting it 4.5 times a
// second; a field instead pads to a fixed width and rewrites only the
// character cells whose content actually changed, so a steady value costs no
// SPI traffic at all.
typedef struct {
    int16_t  x, y;
    uint8_t  w;          // width in characters
    uint8_t  big;        // 0 = 6x8, 1 = 12x16
    uint16_t fg, bg;
    char     cur[44];
} field_t;

// Touch calibration, measured by the guided routine and persisted to NVS.
typedef struct {
    uint16_t x_min, x_max;   // raw range along the screen's X axis
    uint16_t y_min, y_max;   // raw range along the screen's Y axis
    bool     swap;           // controller axes transposed vs the display
    bool     inv_x, inv_y;
    bool     valid;
} touch_cal_t;

// UI screens. SCR_TAB_COUNT marks the end of the tab bar: the procedure
// pages after it are reached only from SETUP, and deliberately have no tab so
// a stray tap cannot abandon a half-explained procedure.
typedef enum { SCR_WELCOME = 0, SCR_SWEEP, SCR_LIST, SCR_DETAIL, SCR_SETUP,
               SCR_TAB_COUNT,
               SCR_BASELINE_INFO, SCR_RSSICAL_INFO,
               SCR_COUNT } screen_t;

// A procedure explainer page.
#define INFO_MAX_LINES 16
typedef struct {
    const char *title;
    const char *lines[INFO_MAX_LINES];
} infopage_t;

// A touch target.
typedef struct { int16_t x, y, w, h; } rect_t;

// A horizontal bar that paints only the segment that changed.
typedef struct { int16_t x, y, w, h; int16_t cur; uint16_t col; } bar_t;


// ---------------------------------------------------------------------------
// SPI BUS ARBITRATION
//
// Three peripherals share this bus - display, touch controller and SD card -
// and they are NOT all driven from the same task. Drawing and touch run on
// the UI task; SD logging runs on the radio pump task, which is higher
// priority and will preempt a half-finished display transaction.
//
// SPI.beginTransaction() alone does not protect against that: it configures
// the bus and, depending on core build options, may not serialise across
// tasks at all. A preempted transfer leaves the display mid-command with CS
// asserted, which shows up as corrupted rows, a frozen panel, or SD writes
// that silently return garbage.
//
// So every SPI user takes this mutex first. It is recursive because the draw
// helpers call each other (draw_text -> draw_char), and it is taken for the
// whole logical operation rather than per byte.
// ---------------------------------------------------------------------------
static SemaphoreHandle_t g_spi_mux = NULL;

static void spi_mux_init()
{
    if (!g_spi_mux) g_spi_mux = xSemaphoreCreateRecursiveMutex();
}

static inline void spi_take()
{
    if (g_spi_mux) xSemaphoreTakeRecursive(g_spi_mux, portMAX_DELAY);
}

static inline void spi_give()
{
    if (g_spi_mux) xSemaphoreGiveRecursive(g_spi_mux);
}

// ===========================================================================
// DISPLAY  -  raw ST7789 / ILI9341 over the stock Arduino SPI library.
//
// Written out longhand rather than using TFT_eSPI because upstream TFT_eSPI
// has no ESP32-C5 backend: it needs the vendor's TFT_eSPI_ESP32_C5.c/h files
// dropped into the library folder plus target-guard edits, which in Arduino
// IDE means hand-patching a managed library that gets wiped on update.
//
// This is about 200 lines and depends only on SPI.h, so the sketch stays
// single-file and Arduino IDE works out of the box.
// ===========================================================================

// 16-bit RGB565
#define RGB(r,g,b) ((uint16_t)((((r)&0xF8)<<8) | (((g)&0xFC)<<3) | ((b)>>3)))
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_DIM     0x7BEF
#define C_GREEN   0x03E0
#define C_AMBER   0xFD20
#define C_RED     0xF800
#define C_CYAN    0x07FF
#define C_YELLOW  0xFFE0
#define C_PANEL   0x2104
#define C_BLUE    0x001F

static SPISettings SPI_LCD(SPI_HZ_LCD, MSBFIRST, SPI_MODE0);
static SPISettings SPI_TCH(SPI_HZ_TOUCH, MSBFIRST, SPI_MODE0);

static bool g_lcd_ok = false;

static inline void lcd_cs_low()  { digitalWrite(PIN_LCD_CS, LOW); }
static inline void lcd_cs_high() { digitalWrite(PIN_LCD_CS, HIGH); }

static void lcd_cmd(uint8_t c)
{
    digitalWrite(PIN_LCD_DC, LOW);
    lcd_cs_low();
    SPI.transfer(c);
    lcd_cs_high();
}

static void lcd_data(const uint8_t *d, size_t n)
{
    if (!n) return;
    digitalWrite(PIN_LCD_DC, HIGH);
    lcd_cs_low();
    for (size_t i = 0; i < n; i++) SPI.transfer(d[i]);
    lcd_cs_high();
}

static void lcd_cmd_data(uint8_t c, const uint8_t *d, size_t n)
{
    lcd_cmd(c);
    lcd_data(d, n);
}

// MADCTL: rotation plus colour order. Bit 3 is the BGR flag on both panels.
static uint8_t madctl_value()
{
#if PANEL_DRIVER == 0     // ST7789
    uint8_t v;
    switch (PANEL_ROT & 3) {
        case 0:  v = 0x00; break;            // 240x320 portrait
        case 1:  v = 0x60; break;            // MV | MX  -> 320x240
        case 2:  v = 0xC0; break;
        default: v = 0xA0; break;            // MV | MY  -> 320x240
    }
#else                     // ILI9341
    uint8_t v;
    switch (PANEL_ROT & 3) {
        case 0:  v = 0x40; break;
        case 1:  v = 0x20; break;
        case 2:  v = 0x80; break;
        default: v = 0xE0; break;
    }
#endif
#if PANEL_BGR
    v |= 0x08;
#endif
    return v;
}

static void lcd_init()
{
    pinMode(PIN_LCD_CS, OUTPUT);  lcd_cs_high();
    pinMode(PIN_LCD_DC, OUTPUT);  digitalWrite(PIN_LCD_DC, HIGH);
#if PIN_LCD_RST >= 0
    pinMode(PIN_LCD_RST, OUTPUT);
    digitalWrite(PIN_LCD_RST, LOW);  delay(20);
    digitalWrite(PIN_LCD_RST, HIGH); delay(150);
#endif

    SPI.beginTransaction(SPI_LCD);

    lcd_cmd(0x01); delay(150);            // SWRESET - needed, no reset line
    lcd_cmd(0x11); delay(120);            // SLPOUT

    uint8_t colmod = 0x55;                // 16 bits/pixel
    lcd_cmd_data(0x3A, &colmod, 1);

    uint8_t mad = madctl_value();
    lcd_cmd_data(0x36, &mad, 1);

    // Inversion. The vendor's TFT_eSPI ST7789 init table sends INVON
    // unconditionally, which is exactly why issue #3 reports red rendering as
    // yellow or cyan on this panel. It wants INVOFF.
#if PANEL_INVERT
    lcd_cmd(0x21);                        // INVON
#else
    lcd_cmd(0x20);                        // INVOFF
#endif

#if PANEL_DRIVER == 1
    // ILI9341 wants a power/timing preamble that the ST7789 does not.
    { uint8_t d[] = {0x23};              lcd_cmd_data(0xC0, d, 1); }  // PWCTR1
    { uint8_t d[] = {0x10};              lcd_cmd_data(0xC1, d, 1); }  // PWCTR2
    { uint8_t d[] = {0x3E, 0x28};        lcd_cmd_data(0xC5, d, 2); }  // VMCTR1
    { uint8_t d[] = {0x86};              lcd_cmd_data(0xC7, d, 1); }  // VMCTR2
    { uint8_t d[] = {0x00, 0x18};        lcd_cmd_data(0xB1, d, 2); }  // FRMCTR1
    { uint8_t d[] = {0x08, 0x82, 0x27};  lcd_cmd_data(0xB6, d, 3); }  // DFUNCTR
#endif

    lcd_cmd(0x13); delay(10);             // NORON
    lcd_cmd(0x29); delay(120);            // DISPON

    SPI.endTransaction();

    pinMode(PIN_BL, OUTPUT);
    digitalWrite(PIN_BL, HIGH);
    g_lcd_ok = true;
}

static void lcd_set_window(int16_t x, int16_t y, int16_t w, int16_t h)
{
    uint16_t x1 = x, x2 = x + w - 1, y1 = y, y2 = y + h - 1;
    uint8_t b[4];
    b[0] = x1 >> 8; b[1] = x1 & 0xFF; b[2] = x2 >> 8; b[3] = x2 & 0xFF;
    lcd_cmd_data(0x2A, b, 4);                       // CASET
    b[0] = y1 >> 8; b[1] = y1 & 0xFF; b[2] = y2 >> 8; b[3] = y2 & 0xFF;
    lcd_cmd_data(0x2B, b, 4);                       // RASET
    lcd_cmd(0x2C);                                  // RAMWR
}

// Push one colour over a rectangle. Chunked so we are not calling transfer()
// per pixel for a full-screen clear.
static void fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t col)
{
    if (!g_lcd_ok) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x >= SCR_W || y >= SCR_H || w <= 0 || h <= 0) return;
    if (x + w > SCR_W) w = SCR_W - x;
    if (y + h > SCR_H) h = SCR_H - y;

    static uint8_t buf[128];                        // 64 pixels
    for (int i = 0; i < 64; i++) { buf[i*2] = col >> 8; buf[i*2+1] = col & 0xFF; }

    spi_take();
    SPI.beginTransaction(SPI_LCD);
    lcd_set_window(x, y, w, h);
    digitalWrite(PIN_LCD_DC, HIGH);
    lcd_cs_low();
    uint32_t left = (uint32_t)w * h;
    while (left) {
        uint32_t n = left > 64 ? 64 : left;
        SPI.writeBytes(buf, n * 2);
        left -= n;
    }
    lcd_cs_high();
    SPI.endTransaction();
    spi_give();
}

static void fill_screen(uint16_t col) { fill_rect(0, 0, SCR_W, SCR_H, col); }

static void draw_hline(int16_t x, int16_t y, int16_t w, uint16_t c)
{ fill_rect(x, y, w, 1, c); }
static void draw_vline(int16_t x, int16_t y, int16_t h, uint16_t c)
{ fill_rect(x, y, 1, h, c); }

static void draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c)
{
    draw_hline(x, y, w, c);
    draw_hline(x, y + h - 1, w, c);
    draw_vline(x, y, h, c);
    draw_vline(x + w - 1, y, h, c);
}

// ---------------------------------------------------------------------------
// FONTS  -  two sizes, both column-major, bit0 = top row.
//
// Generated from DejaVu Sans Mono REGULAR and verified glyph by glyph.
// The first build of this firmware used the BOLD face, which puts 2px stems
// in a 6px cell and reads as heavy and smeared on this panel. Regular gives
// clean 1px stems at 6x8.
//
// FONT6x8   body text, lists, labels. Used at scale 1 almost everywhere.
// FONT12x16 headings and the big RSSI readout. A real 12x16 face rather than
//           6x8 scaled 2x, which is what made the old readout look blocky.
// ---------------------------------------------------------------------------

static const uint8_t FONT6x8[95][6] = {
  {0x00,0x00,0x00,0x00,0x00,0x00}, //  
  {0x00,0x00,0x9E,0xBE,0x00,0x00}, // !
  {0x00,0x00,0x0E,0x0E,0x00,0x00}, // "
  {0x20,0xE8,0x3E,0xE8,0x2E,0x28}, // #
  {0x00,0x9C,0x94,0xB4,0xE4,0x00}, // $
  {0x04,0x2A,0x1E,0xF0,0xA8,0x40}, // %
  {0x60,0xDC,0x9A,0xB2,0xE0,0xB0}, // &
  {0x00,0x00,0x00,0x00,0x00,0x00}, // apostrophe
  {0x00,0x00,0x7C,0x83,0x00,0x00}, // (
  {0x00,0x00,0x83,0x7C,0x00,0x00}, // )
  {0x00,0x12,0x0C,0x0C,0x12,0x00}, // *
  {0x10,0x10,0x10,0x10,0x10,0x10}, // +
  {0x00,0x00,0x80,0x80,0x00,0x00}, // ,
  {0x00,0x00,0x20,0x20,0x00,0x00}, // -
  {0x00,0x00,0x80,0x80,0x00,0x00}, // .
  {0x00,0x80,0x60,0x18,0x06,0x00}, // /
  {0x00,0x7C,0x92,0x92,0x7C,0x00}, // 0
  {0x00,0x82,0x82,0xFE,0x80,0x00}, // 1
  {0x00,0xC6,0xA2,0x92,0x8C,0x00}, // 2
  {0x00,0x86,0x92,0x92,0xEC,0x00}, // 3
  {0x20,0x30,0x24,0x26,0xFE,0x20}, // 4
  {0x00,0x8E,0x8A,0x8A,0x72,0x00}, // 5
  {0x00,0x7C,0x92,0x92,0xF2,0x00}, // 6
  {0x00,0x02,0xC2,0x3A,0x0E,0x00}, // 7
  {0x00,0xEE,0x92,0x92,0xFE,0x00}, // 8
  {0x00,0x9E,0x92,0x92,0x7C,0x00}, // 9
  {0x00,0x00,0x88,0x88,0x00,0x00}, // :
  {0x00,0x00,0x88,0x88,0x00,0x00}, // ;
  {0x10,0x10,0x28,0x28,0x44,0x44}, // <
  {0x28,0x28,0x28,0x28,0x28,0x28}, // =
  {0x44,0x44,0x28,0x28,0x10,0x10}, // >
  {0x00,0x02,0xA2,0x1A,0x0E,0x00}, // ?
  {0xF8,0x04,0x72,0x8A,0x8A,0xFC}, // @
  {0x80,0x70,0x2E,0x26,0x70,0x80}, // A
  {0x00,0xFE,0x92,0x92,0xFE,0x40}, // B
  {0x00,0x7C,0x82,0x82,0x82,0x00}, // C
  {0x00,0xFE,0x82,0x82,0x7C,0x00}, // D
  {0x00,0xFE,0x92,0x92,0x92,0x00}, // E
  {0x00,0xFE,0x12,0x12,0x12,0x02}, // F
  {0x10,0x7C,0x82,0x92,0xF2,0x00}, // G
  {0x00,0xFE,0x10,0x10,0xFE,0x00}, // H
  {0x00,0x82,0xFE,0xFE,0x82,0x00}, // I
  {0x00,0x80,0x82,0xC2,0x7E,0x00}, // J
  {0x00,0xFE,0x18,0x3C,0xC2,0x80}, // K
  {0x00,0xFE,0x80,0x80,0x80,0x80}, // L
  {0xFE,0x06,0x18,0x18,0x06,0xFE}, // M
  {0x00,0xFE,0x0C,0x60,0xFE,0x00}, // N
  {0x00,0x7C,0x82,0x82,0x7C,0x00}, // O
  {0x00,0xFE,0x12,0x12,0x1E,0x0C}, // P
  {0x00,0x7C,0x82,0x82,0xFC,0x00}, // Q
  {0x00,0xFE,0x12,0x12,0x6E,0x80}, // R
  {0x00,0xCE,0x92,0x92,0xE4,0x00}, // S
  {0x02,0x02,0xFE,0xFE,0x02,0x02}, // T
  {0x00,0xFE,0x80,0x80,0xFE,0x00}, // U
  {0x02,0x1C,0xE0,0xE0,0x1C,0x02}, // V
  {0x1E,0xE0,0x38,0x38,0xE0,0x1E}, // W
  {0x80,0xC6,0x38,0x38,0x46,0x82}, // X
  {0x02,0x06,0xF8,0xF8,0x06,0x02}, // Y
  {0x00,0xC2,0xA2,0x9A,0x86,0x82}, // Z
  {0x00,0x00,0xFF,0x01,0x00,0x00}, // [
  {0x00,0x06,0x18,0x60,0x80,0x00}, // backslash
  {0x00,0x00,0x01,0xFF,0x00,0x00}, // ]
  {0x00,0x0C,0x06,0x06,0x0C,0x00}, // ^
  {0x00,0x00,0x00,0x00,0x00,0x00}, // _
  {0x00,0x00,0x03,0x00,0x00,0x00}, // `
  {0x00,0xE8,0xA8,0xA8,0xF8,0x00}, // a
  {0x00,0xFF,0x88,0x88,0xF8,0x00}, // b
  {0x00,0x70,0x88,0x88,0x88,0x00}, // c
  {0x00,0xF8,0x88,0x88,0xFF,0x00}, // d
  {0x00,0x70,0xA8,0xA8,0xB8,0x00}, // e
  {0x00,0x08,0xFE,0x0B,0x09,0x00}, // f
  {0x00,0xF8,0x88,0x88,0xF8,0x00}, // g
  {0x00,0xFF,0x08,0x08,0xF8,0x00}, // h
  {0x00,0x88,0x88,0xF9,0x80,0x00}, // i
  {0x00,0x08,0x08,0xF9,0x00,0x00}, // j
  {0x00,0xFF,0x20,0x30,0xC8,0x80}, // k
  {0x00,0x01,0x7F,0x80,0x80,0x00}, // l
  {0xF8,0x18,0x18,0xF8,0x08,0xF0}, // m
  {0x00,0xF8,0x08,0x08,0xF8,0x00}, // n
  {0x00,0xF8,0x88,0x88,0xF8,0x00}, // o
  {0x00,0xF8,0x88,0x88,0xF8,0x00}, // p
  {0x00,0xF8,0x88,0x88,0xF8,0x00}, // q
  {0x00,0x00,0xF8,0x08,0x08,0x08}, // r
  {0x00,0x98,0xA8,0xA8,0xC8,0x00}, // s
  {0x00,0x08,0xFE,0x88,0x88,0x00}, // t
  {0x00,0xF8,0x80,0x80,0xF8,0x00}, // u
  {0x00,0x38,0xC0,0xC0,0x38,0x00}, // v
  {0x18,0xE0,0x60,0x60,0xE0,0x18}, // w
  {0x00,0x88,0x70,0x70,0x88,0x00}, // x
  {0x00,0x18,0xE0,0xC0,0x38,0x00}, // y
  {0x00,0x88,0x88,0x88,0x88,0x00}, // z
  {0x00,0x10,0x38,0xC7,0x01,0x00}, // {
  {0x00,0x00,0x00,0x00,0x00,0x00}, // |
  {0x00,0x01,0xC7,0x38,0x10,0x00}, // }
  {0x00,0x10,0x10,0x20,0x20,0x00}, // ~
};

// 12x16: two bytes per column, low byte = rows 0-7, high byte = rows 8-15.
static const uint16_t FONT12x16[95][12] = {
  {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000}, //  
  {0x0000,0x0000,0x0000,0x0000,0x0000,0xCFFE,0xCFFE,0x0000,0x0000,0x0000,0x0000,0x0000}, // !
  {0x0000,0x0000,0x0000,0x003E,0x003E,0x0000,0x0000,0x003E,0x003E,0x0000,0x0000,0x0000}, // "
  {0x0C00,0x0C60,0xFC60,0x7F60,0x0FF8,0x0C7E,0xFC62,0x3FE0,0x0FF8,0x0C7E,0x0C62,0x0060}, // #
  {0x0000,0x0000,0x61F0,0xC1F0,0xC318,0xC318,0xFFFE,0xC618,0xE618,0x7C30,0x3800,0x0000}, // $
  {0x0018,0x043C,0x0442,0x0242,0x0266,0x013C,0x7918,0xCC80,0x8480,0x8440,0x7840,0x3000}, // %
  {0x0000,0x3E00,0x7338,0xE1FC,0xC1CE,0xC386,0xC606,0xCC06,0x7806,0x7000,0xFE00,0x8600}, // &
  {0x0000,0x0000,0x0000,0x0000,0x0000,0x003E,0x003E,0x0000,0x0000,0x0000,0x0000,0x0000}, // apostrophe
  {0x0000,0x0000,0x0000,0x0000,0x1F80,0xFFF0,0xE07C,0x0006,0x0000,0x0000,0x0000,0x0000}, // (
  {0x0000,0x0000,0x0000,0x0000,0x0006,0xE07C,0xFFF0,0x1F80,0x0000,0x0000,0x0000,0x0000}, // )
  {0x0000,0x0000,0x0198,0x0090,0x00F0,0x07FE,0x07FE,0x00F0,0x0090,0x0198,0x0000,0x0000}, // *
  {0x0000,0x0600,0x0600,0x0600,0x0600,0x7FE0,0x7FE0,0x0600,0x0600,0x0600,0x0600,0x0000}, // +
  {0x0000,0x0000,0x0000,0x0000,0x0000,0xE000,0xE000,0x0000,0x0000,0x0000,0x0000,0x0000}, // ,
  {0x0000,0x0000,0x0000,0x0600,0x0600,0x0600,0x0600,0x0600,0x0600,0x0000,0x0000,0x0000}, // -
  {0x0000,0x0000,0x0000,0x0000,0x0000,0xE000,0xE000,0x0000,0x0000,0x0000,0x0000,0x0000}, // .
  {0x0000,0x0000,0xC000,0xF000,0x3C00,0x0F80,0x03E0,0x0078,0x001E,0x0006,0x0000,0x0000}, // /
  {0x0000,0x0FE0,0x3FF8,0x783C,0xE00E,0xC306,0xC306,0xE00E,0x783C,0x3FF8,0x0FE0,0x0000}, // 0
  {0x0000,0x0000,0x000C,0xC00C,0xC006,0xC006,0xFFFE,0xFFFE,0xC000,0xC000,0xC000,0x0000}, // 1
  {0x0000,0xC000,0xE00C,0xF006,0xD806,0xCC06,0xC606,0xC38E,0xC1FC,0xC0F8,0x0000,0x0000}, // 2
  {0x0000,0x6000,0x600C,0xC006,0xC186,0xC186,0xC186,0xE3CE,0x7FFC,0x7E78,0x1800,0x0000}, // 3
  {0x0000,0x1C00,0x1F00,0x1980,0x18E0,0x1838,0x180E,0xFFFE,0xFFFE,0x1800,0x1800,0x0000}, // 4
  {0x0000,0x6000,0xE1FE,0xC0FE,0xC0C6,0xC0C6,0xC0C6,0xE1C6,0x7F86,0x3F00,0x0000,0x0000}, // 5
  {0x0000,0x0FC0,0x3FF8,0x7F3C,0xE30E,0xC186,0xC186,0xC186,0xE386,0x7F0C,0x1C00,0x0000}, // 6
  {0x0000,0x0006,0x0006,0x8006,0xE006,0xFC06,0x1F06,0x03E6,0x00FE,0x001E,0x0002,0x0000}, // 7
  {0x0000,0x1C00,0x7E7C,0xF7FC,0xC186,0xC186,0xC186,0xC186,0xF7FC,0x7E7C,0x3C00,0x0000}, // 8
  {0x0000,0x0070,0x61FC,0xC38E,0xC306,0xC306,0xC306,0xE18E,0x79FC,0x3FF8,0x07E0,0x0000}, // 9
  {0x0000,0x0000,0x0000,0x0000,0x0000,0xE1C0,0xE1C0,0x0000,0x0000,0x0000,0x0000,0x0000}, // :
  {0x0000,0x0000,0x0000,0x0000,0x0000,0xE1C0,0xE1C0,0x0000,0x0000,0x0000,0x0000,0x0000}, // ;
  {0x0000,0x0600,0x0F00,0x0F00,0x0900,0x1980,0x1980,0x30C0,0x30C0,0x30C0,0x6060,0x0000}, // <
  {0x0000,0x1980,0x1980,0x1980,0x1980,0x1980,0x1980,0x1980,0x1980,0x1980,0x1980,0x0000}, // =
  {0x0000,0x6060,0x30C0,0x30C0,0x30C0,0x1980,0x1980,0x0900,0x0F00,0x0F00,0x0600,0x0000}, // >
  {0x0000,0x0000,0x000C,0x000E,0x0006,0xDE06,0xDF86,0x01CE,0x00FC,0x007C,0x0000,0x0000}, // ?
  {0x1F00,0xFFE0,0xC030,0x0018,0x1F0C,0x3F84,0x60C4,0x4044,0x404C,0x60D8,0x7FF8,0x7FC0}, // @
  {0x8000,0xF000,0xFE00,0x1FE0,0x19FC,0x181E,0x181E,0x19FC,0x1FE0,0xFF00,0xF000,0x8000}, // A
  {0x0000,0x0000,0xFFFE,0xFFFE,0xC186,0xC186,0xC186,0xC186,0xE3FE,0x7E7C,0x3C30,0x0000}, // B
  {0x0000,0x07C0,0x1FF0,0x7EFC,0x600C,0xC006,0xC006,0xC006,0xC006,0xE00E,0x4004,0x0000}, // C
  {0x0000,0xFFFE,0xFFFE,0xC006,0xC006,0xC006,0xC006,0x600C,0x783C,0x3FF8,0x0FE0,0x0000}, // D
  {0x0000,0x0000,0xFFFE,0xFFFE,0xC186,0xC186,0xC186,0xC186,0xC186,0xC186,0xC006,0x0000}, // E
  {0x0000,0x0000,0xFFFE,0xFFFE,0x0186,0x0186,0x0186,0x0186,0x0186,0x0186,0x0006,0x0000}, // F
  {0x0000,0x0FE0,0x3FF8,0x783C,0xE00E,0xC006,0xC006,0xC306,0xC306,0x7F0C,0x3F00,0x0000}, // G
  {0x0000,0xFFFE,0xFFFE,0x0180,0x0180,0x0180,0x0180,0x0180,0x0180,0xFFFE,0xFFFE,0x0000}, // H
  {0x0000,0x0000,0xC006,0xC006,0xC006,0xFFFE,0xFFFE,0xC006,0xC006,0xC006,0x0000,0x0000}, // I
  {0x0000,0x6000,0xC000,0xC000,0xC006,0xC006,0xE006,0xFFFE,0x7FFE,0x0000,0x0000,0x0000}, // J
  {0x0000,0xFFFE,0xFFFE,0x0380,0x01C0,0x03E0,0x0770,0x1E38,0x381C,0xF00E,0xC002,0x8000}, // K
  {0x0000,0x0000,0xFFFE,0xFFFE,0xC000,0xC000,0xC000,0xC000,0xC000,0xC000,0xC000,0x0000}, // L
  {0x0000,0xFFFE,0xFFFE,0x003E,0x01F0,0x0780,0x0780,0x01F0,0x003E,0xFFFE,0xFFFE,0x0000}, // M
  {0x0000,0xFFFE,0xFFFE,0x001E,0x007C,0x01E0,0x0F80,0x3C00,0xF000,0xFFFE,0xFFFE,0x0000}, // N
  {0x0000,0x0FE0,0x3FF8,0x783C,0xE00E,0xC006,0xC006,0xE00E,0x701C,0x3FF8,0x0FE0,0x0000}, // O
  {0x0000,0x0000,0xFFFE,0xFFFE,0x0306,0x0306,0x0306,0x0306,0x038E,0x01FC,0x00F8,0x0000}, // P
  {0x0000,0x0FE0,0x3FF8,0x783C,0xE00E,0xC006,0xC006,0xE00E,0xF01C,0x3FF8,0x0FE0,0x0000}, // Q
  {0x0000,0xFFFE,0xFFFE,0x0186,0x0186,0x0186,0x0186,0x07CE,0x1EFC,0x7C7C,0xF000,0x8000}, // R
  {0x0000,0x4070,0x60FC,0xC1DC,0xC186,0xC186,0xC306,0xC306,0xE706,0x7E0C,0x1C00,0x0000}, // S
  {0x0006,0x0006,0x0006,0x0006,0x0006,0xFFFE,0xFFFE,0x0006,0x0006,0x0006,0x0006,0x0006}, // T
  {0x0000,0x0FFE,0x7FFE,0x7800,0xC000,0xC000,0xC000,0xC000,0x7000,0x7FFE,0x1FFE,0x0000}, // U
  {0x0000,0x001E,0x00FE,0x0FF0,0x7F00,0xF000,0xF000,0x7F00,0x0FF0,0x00FE,0x001E,0x0000}, // V
  {0x001E,0x0FFE,0xFFE0,0xF800,0x3F80,0x03E0,0x03E0,0x3F80,0xF800,0xFFC0,0x0FFE,0x001E}, // W
  {0x8000,0xC002,0xF00E,0x3C3C,0x0E70,0x07E0,0x03C0,0x0FF0,0x3C3C,0xF00E,0xC006,0x8000}, // X
  {0x0000,0x0006,0x001E,0x0078,0x00E0,0xFF80,0xFF80,0x00E0,0x0078,0x001E,0x0006,0x0000}, // Y
  {0x0000,0x8000,0xE006,0xF806,0xDC06,0xC706,0xC386,0xC0E6,0xC076,0xC01E,0xC00E,0x0000}, // Z
  {0x0000,0x0000,0x0000,0x0000,0xFFFE,0xFFFE,0x0006,0x0006,0x0006,0x0000,0x0000,0x0000}, // [
  {0x0000,0x0002,0x000E,0x003C,0x00F0,0x07C0,0x1F00,0x7800,0xE000,0x8000,0x0000,0x0000}, // backslash
  {0x0000,0x0000,0x0000,0x0006,0x0006,0x0006,0xFFFE,0xFFFE,0x0000,0x0000,0x0000,0x0000}, // ]
  {0x0000,0x0020,0x0030,0x0018,0x000C,0x0006,0x0006,0x000C,0x0018,0x0030,0x0020,0x0000}, // ^
  {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000}, // _
  {0x0000,0x0000,0x0000,0x0001,0x0003,0x000E,0x0008,0x0000,0x0000,0x0000,0x0000,0x0000}, // `
  {0x0000,0x3800,0x7CC0,0xEEE0,0xC660,0xC660,0xC660,0x6660,0xFFC0,0xFFC0,0x0000,0x0000}, // a
  {0x0000,0x0000,0xFFFE,0xFFFE,0x60C0,0xC060,0xC060,0xC060,0xE0E0,0x7FC0,0x1F00,0x0000}, // b
  {0x0000,0x0000,0x1F00,0x7FC0,0x60C0,0xC060,0xC060,0xC060,0xC060,0xE0E0,0x0000,0x0000}, // c
  {0x0000,0x1F00,0x7FC0,0xF1E0,0xC060,0xC060,0xC060,0x60C0,0xFFFE,0xFFFE,0x0000,0x0000}, // d
  {0x0000,0x1F00,0x3F80,0x76C0,0xE660,0xC660,0xC660,0xC660,0xC6E0,0x67C0,0x4700,0x0000}, // e
  {0x0000,0x0000,0x0060,0x0060,0x0060,0xFFFC,0xFFFE,0x0066,0x0066,0x0066,0x0000,0x0000}, // f
  {0x0000,0x1F00,0x7FC0,0xF1E0,0xC060,0xC060,0xC060,0x60C0,0xFFE0,0xFFE0,0x0000,0x0000}, // g
  {0x0000,0x0000,0xFFFE,0xFFFE,0x00C0,0x0060,0x0060,0x0060,0xFFE0,0xFFC0,0x0000,0x0000}, // h
  {0x0000,0x0000,0xC060,0xC060,0xC060,0xFFE6,0xFFE6,0xC000,0xC000,0xC000,0xC000,0x0000}, // i
  {0x0000,0x0000,0x0000,0x0060,0x0060,0x0060,0xFFE6,0xFFE6,0x0000,0x0000,0x0000,0x0000}, // j
  {0x0000,0x0000,0xFFFE,0xFFFE,0x0600,0x0700,0x0F80,0x1DC0,0x78E0,0xE060,0xC020,0x0000}, // k
  {0x0000,0x0006,0x0006,0x0006,0x1FFE,0x7FFE,0xE000,0xC000,0xC000,0xC000,0x0000,0x0000}, // l
  {0x0000,0xFFE0,0xFFE0,0x0060,0x0060,0xFFE0,0xFFC0,0x0060,0x0060,0xFFE0,0xFFC0,0x0000}, // m
  {0x0000,0x0000,0xFFE0,0xFFE0,0x00C0,0x0060,0x0060,0x0060,0xFFE0,0xFFC0,0x0000,0x0000}, // n
  {0x0000,0x0E00,0x7FC0,0x71C0,0xE0E0,0xC060,0xC060,0xE0E0,0x71C0,0x7FC0,0x1F00,0x0000}, // o
  {0x0000,0x0000,0xFFE0,0xFFE0,0x60C0,0xC060,0xC060,0xC060,0xF1E0,0x7FC0,0x1F00,0x0000}, // p
  {0x0000,0x0E00,0x7FC0,0x71C0,0xC060,0xC060,0xC060,0x60C0,0xFFE0,0xFFE0,0x0000,0x0000}, // q
  {0x0000,0x0000,0x0000,0xFFE0,0xFFE0,0x0380,0x00C0,0x0060,0x0060,0x0060,0x00E0,0x0000}, // r
  {0x0000,0x0000,0x6380,0xE3C0,0xC6E0,0xC460,0xC460,0xCC60,0x7CE0,0x78C0,0x0000,0x0000}, // s
  {0x0000,0x0060,0x0060,0x0060,0x3FFC,0xFFFC,0xE060,0xC060,0xC060,0xC060,0x0000,0x0000}, // t
  {0x0000,0x0000,0x7FE0,0xFFE0,0xE000,0xC000,0xC000,0x6000,0xFFE0,0xFFE0,0x0000,0x0000}, // u
  {0x0000,0x0060,0x03E0,0x0F80,0x7C00,0xE000,0xE000,0x7C00,0x0F80,0x03E0,0x0060,0x0000}, // v
  {0x0060,0x0FE0,0xFF00,0xE000,0x7800,0x0F00,0x0F00,0x7C00,0xE000,0xFF00,0x0FE0,0x00E0}, // w
  {0x0000,0x8020,0xC060,0x70E0,0x3B80,0x0F00,0x0F00,0x3B80,0x70E0,0xC060,0x8020,0x0000}, // x
  {0x0000,0x0020,0x01E0,0x07C0,0x3E00,0xF000,0xE000,0x7C00,0x0F80,0x01E0,0x0060,0x0000}, // y
  {0x0000,0x0000,0xC060,0xE060,0xD860,0xCC60,0xC660,0xC360,0xC1E0,0xC060,0x0000,0x0000}, // z
  {0x0000,0x0000,0x0600,0x0600,0x0F00,0xFFFC,0xF9FE,0x000E,0x0006,0x0006,0x0000,0x0000}, // {
  {0x0000,0x0000,0x0000,0x0000,0x0000,0xFFFE,0xFFFE,0x0000,0x0000,0x0000,0x0000,0x0000}, // |
  {0x0000,0x0000,0x0006,0x0006,0x0006,0xF9FE,0xFFFC,0x0F00,0x0600,0x0600,0x0000,0x0000}, // }
  {0x0000,0x0600,0x0300,0x0300,0x0300,0x0300,0x0600,0x0600,0x0600,0x0600,0x0300,0x0000}, // ~
};

// Draw one character at integer scale. Glyph is column-major, bit0 = top row.
// Fold to upper case at the single point every glyph passes through, so
// captured strings are covered without touching any call site.
static inline char ui_fold(char c)
{
#if UI_ALL_CAPS
    if (c >= 'a' && c <= 'z') return (char)(c - 32);
#endif
    return c;
}

static void draw_char(int16_t x, int16_t y, char ch, uint16_t fg, uint16_t bg,
                      uint8_t scale)
{
    if (!g_lcd_ok) return;
    ch = ui_fold(ch);
    if (ch < 32 || ch > 126) ch = '?';
    const uint8_t *g = FONT6x8[ch - 32];

    // Build the glyph into a pixel buffer and push it in one window write -
    // far fewer SPI transactions than per-pixel plotting.
    const int cw = 6 * scale, chh = 8 * scale;
    if (cw * chh > 6 * 8 * 16) return;              // scale cap: 4

    static uint8_t px[6 * 8 * 16 * 2];
    int idx = 0;
    for (int row = 0; row < chh; row++) {
        int sy = row / scale;
        for (int colp = 0; colp < cw; colp++) {
            int sx = colp / scale;
            uint16_t c = ((g[sx] >> sy) & 1) ? fg : bg;
            px[idx++] = c >> 8;
            px[idx++] = c & 0xFF;
        }
    }

    spi_take();
    SPI.beginTransaction(SPI_LCD);
    lcd_set_window(x, y, cw, chh);
    digitalWrite(PIN_LCD_DC, HIGH);
    lcd_cs_low();
    SPI.writeBytes(px, idx);
    lcd_cs_high();
    SPI.endTransaction();
    spi_give();
}

// Body text, always 6x8 at scale 1. No scale parameter and no default
// argument: the Arduino preprocessor injects a prototype for every function,
// and a default argument specified in both the injected prototype and the
// definition is a compile error. Headings use draw_text_12 instead.
static void draw_text(int16_t x, int16_t y, const char *s, uint16_t fg, uint16_t bg)
{
    if (!s) return;
    int16_t cx = x;
    spi_take();                     // hold the bus for the whole string
    while (*s) {
        if (cx + 6 > SCR_W) break;
        draw_char(cx, y, *s++, fg, bg, 1);
        cx += 6;
    }
    spi_give();
}

// ---------------------------------------------------------------------------
// 12x16 text. A real face rather than 6x8 scaled up, which is what made the
// first build's readout look blocky. Two bytes per column: low byte rows 0-7,
// high byte rows 8-15.
// ---------------------------------------------------------------------------
static void draw_char_12(int16_t x, int16_t y, char ch, uint16_t fg, uint16_t bg)
{
    if (!g_lcd_ok) return;
    ch = ui_fold(ch);
    if (ch < 32 || ch > 126) ch = '?';
    const uint16_t *g = FONT12x16[ch - 32];

    static uint8_t px[12 * 16 * 2];
    int idx = 0;
    for (int row = 0; row < 16; row++)
        for (int col = 0; col < 12; col++) {
            uint16_t c = ((g[col] >> row) & 1) ? fg : bg;
            px[idx++] = c >> 8;
            px[idx++] = c & 0xFF;
        }

    spi_take();
    SPI.beginTransaction(SPI_LCD);
    lcd_set_window(x, y, 12, 16);
    digitalWrite(PIN_LCD_DC, HIGH);
    lcd_cs_low();
    SPI.writeBytes(px, idx);
    lcd_cs_high();
    SPI.endTransaction();
    spi_give();
}

static void draw_text_12(int16_t x, int16_t y, const char *s, uint16_t fg, uint16_t bg)
{
    if (!s) return;
    int16_t cx = x;
    spi_take();                     // hold the bus for the whole string
    while (*s) {
        if (cx + 12 > SCR_W) break;
        draw_char_12(cx, y, *s++, fg, bg);
        cx += 12;
    }
    spi_give();
}

static void draw_text_12_c(int16_t cx, int16_t y, const char *s, uint16_t fg, uint16_t bg)
{
    int w = (int)strlen(s) * 12;
    draw_text_12((int16_t)(cx - w / 2), y, s, fg, bg);
}

// ---------------------------------------------------------------------------
// FLICKER-FREE FIELDS
//
// The first build cleared a region and repainted it on every frame, 4.5 times
// a second, which is exactly what the operator saw as flicker. The clears
// were never necessary: draw_char already writes background pixels for the
// off-bits of every glyph cell, so text overwrites cleanly in place.
//
// A field therefore:
//   * pads its content to a fixed character width, so a value that gets
//     SHORTER erases its own tail instead of needing a clear
//   * remembers what is currently on the glass and rewrites only the
//     character cells whose content changed
//
// A steady reading costs zero SPI traffic. A changing RSSI costs four or five
// glyph cells. Chrome - banners, labels, button frames, the tab bar - is
// painted once when the screen changes and never touched again.
//
// That last point matters beyond appearance: it cuts SPI traffic by roughly
// an order of magnitude, which reduces contention with SD logging on the
// shared bus.
// ---------------------------------------------------------------------------

static void field_init(field_t *f, int16_t x, int16_t y, uint8_t w, uint8_t big,
                       uint16_t fg, uint16_t bg)
{
    f->x = x; f->y = y;
    f->w = (w > sizeof(f->cur) - 1) ? (uint8_t)(sizeof(f->cur) - 1) : w;
    f->big = big;
    f->fg = fg; f->bg = bg;
    memset(f->cur, 0, sizeof(f->cur));
}

// Force the next field_set to repaint everything, e.g. after a screen change
// has wiped the glass underneath.
static void field_invalidate(field_t *f)
{
    memset(f->cur, 0, sizeof(f->cur));
}

static void field_set_col(field_t *f, const char *s, uint16_t fg)
{
    char want[sizeof(f->cur)];
    uint8_t n = 0;
    while (s && s[n] && n < f->w) { want[n] = s[n]; n++; }
    while (n < f->w) want[n++] = ' ';       // pad: the tail erases itself
    want[n] = 0;

    bool recolour = (fg != f->fg);
    if (recolour) f->fg = fg;

    const int cw = f->big ? 12 : 6;
    for (uint8_t i = 0; i < f->w; i++) {
        if (!recolour && f->cur[i] == want[i]) continue;   // already correct
        if (f->big) draw_char_12((int16_t)(f->x + i * cw), f->y, want[i], f->fg, f->bg);
        else        draw_char((int16_t)(f->x + i * cw), f->y, want[i], f->fg, f->bg, 1);
    }
    memcpy(f->cur, want, (size_t)f->w + 1);
}

static void field_set(field_t *f, const char *s)
{
    field_set_col(f, s, f->fg);
}

// A horizontal bar that only ever paints the segment that changed (bar_t is
// declared in the TYPE DEFINITIONS block). Repainting the whole bar every
// frame is the second-biggest flicker source after the full-region clears.
static void bar_init(bar_t *b, int16_t x, int16_t y, int16_t w, int16_t h)
{
    b->x = x; b->y = y; b->w = w; b->h = h; b->cur = 0; b->col = C_GREEN;
}

static void bar_set(bar_t *b, int16_t fill, uint16_t col)
{
    if (fill < 0) fill = 0;
    if (fill > b->w) fill = b->w;

    if (col != b->col) {
        // Colour changed, so the existing filled part is wrong: repaint it
        // and nothing else.
        b->col = col;
        if (fill) fill_rect(b->x, b->y, fill, b->h, col);
        if (fill < b->cur) fill_rect(b->x + fill, b->y, b->cur - fill, b->h, C_BLACK);
        b->cur = fill;
        return;
    }
    if (fill > b->cur) fill_rect(b->x + b->cur, b->y, fill - b->cur, b->h, col);
    else if (fill < b->cur) fill_rect(b->x + fill, b->y, b->cur - fill, b->h, C_BLACK);
    b->cur = fill;
}

// Centred 6x8, for button labels.
static void draw_text_c(int16_t cx, int16_t y, const char *s, uint16_t fg, uint16_t bg)
{
    int w = (int)strlen(s) * 6;
    draw_text((int16_t)(cx - w / 2), y, s, fg, bg);
}

// Boot self-test: labelled colour bars. Two seconds to confirm the pins are
// right and whether PANEL_BGR needs flipping, instead of guessing later.
static void lcd_selftest()
{
    fill_screen(C_BLACK);
    fill_rect(0,   0, 106, 90, C_RED);
    fill_rect(106, 0, 107, 90, C_GREEN);
    fill_rect(213, 0, 107, 90, C_BLUE);
    draw_text_12(46,  36, "R", C_WHITE, C_RED);
    draw_text_12(152, 36, "G", C_WHITE, C_GREEN);
    draw_text_12(259, 36, "B", C_WHITE, C_BLUE);
    draw_text(6,  100, "Bars must read R G B left to right.", C_WHITE, C_BLACK);
    draw_text(6,  114, "If R and B are swapped: PANEL_BGR 0", C_AMBER, C_BLACK);
    draw_text(6,  128, "If colours look odd: PANEL_INVERT 1", C_AMBER, C_BLACK);
    draw_text_12(6, 150, "VULPECULA", C_CYAN, C_BLACK);
    delay(2200);
}

// ===========================================================================
// TOUCH  -  XPT2046 resistive, shares the SPI bus, own CS on GPIO 1
//
// The first build of this firmware mapped raw to screen like this:
//
//     mx = map(ax, X_MIN, X_MAX, 0, W-1);
//     my = map(ay, Y_MIN, Y_MAX, 0, H-1);
//     if (swap) { long t = mx; mx = my * W / H; my = t * H / W; }
//
// which is wrong twice over. It mapped each raw axis against the wrong screen
// extent and THEN swapped and rescaled, compounding the error, so touches
// landed nowhere near the finger. The axes must be swapped in the RAW domain,
// before either is mapped, because on this panel the controller's X axis runs
// along the display's Y when the panel is rotated to landscape.
//
// It also relied on hard-coded raw limits. Resistive panels vary enough
// between units that guessed constants are not worth having, so the mapping
// is now MEASURED by a guided 4-point routine and persisted to NVS. The
// routine also determines the swap and both inversions empirically rather
// than asking anyone to guess which of the eight orientations applies.
// ===========================================================================

static touch_cal_t g_tcal;
static volatile uint16_t g_touch_raw_x = 0, g_touch_raw_y = 0, g_touch_raw_z = 0;

static const char *TCAL_KEY = "touch_cal";

static void touch_cal_defaults()
{
    // Deliberately marked invalid: an uncalibrated build must run the guided
    // routine rather than quietly using numbers nobody measured. These values
    // are only a fallback so the UI is reachable if NVS fails.
    g_tcal.x_min = 300;  g_tcal.x_max = 3800;
    g_tcal.y_min = 300;  g_tcal.y_max = 3800;
    g_tcal.swap  = true; g_tcal.inv_x = false; g_tcal.inv_y = true;
    g_tcal.valid = false;
}

static void touch_cal_load()
{
    touch_cal_defaults();
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t len = sizeof(touch_cal_t);
    touch_cal_t st;
    if (nvs_get_blob(h, TCAL_KEY, &st, &len) == ESP_OK && len == sizeof(st)) {
        // Reject a stored blob whose spans are too small to be a real
        // calibration - a degenerate range would make every touch land in one
        // corner, which is worse than falling back to the guided routine.
        if (st.x_max > st.x_min + 400 && st.y_max > st.y_min + 400) g_tcal = st;
    }
    nvs_close(h);
}

static void touch_cal_save()
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, TCAL_KEY, &g_tcal, sizeof(touch_cal_t));
    nvs_commit(h);
    nvs_close(h);
}

static uint16_t xpt_xfer(uint8_t cmd)
{
    SPI.transfer(cmd);
    uint8_t hi = SPI.transfer(0x00);
    uint8_t lo = SPI.transfer(0x00);
    return (uint16_t)(((hi << 8) | lo) >> 3);      // 12-bit result
}

// Raw read with pressure gating and median-of-5 on each axis. A single XPT2046
// sample is noisy enough to jump tens of counts, which on a 320px axis is
// several pixels of jitter - enough to make a button feel unreliable.
static bool touch_raw(uint16_t *rx, uint16_t *ry)
{
    uint16_t xs[5], ys[5];

    spi_take();
    SPI.beginTransaction(SPI_TCH);
    digitalWrite(PIN_TOUCH_CS, LOW);

    uint16_t z1 = xpt_xfer(0xB1);
    uint16_t z2 = xpt_xfer(0xC1);
    uint16_t z  = (uint16_t)(z1 + 4095 - z2);

    for (int i = 0; i < 5; i++) {
        ys[i] = xpt_xfer(0x91);
        xs[i] = xpt_xfer(0xD1);
    }

    digitalWrite(PIN_TOUCH_CS, HIGH);
    SPI.endTransaction();
    spi_give();

    // median of 5, insertion sort
    for (int i = 1; i < 5; i++) {
        uint16_t a = xs[i], b = ys[i]; int j = i - 1;
        while (j >= 0 && xs[j] > a) { xs[j+1] = xs[j]; j--; } xs[j+1] = a;
        j = i - 1;
        while (j >= 0 && ys[j] > b) { ys[j+1] = ys[j]; j--; } ys[j+1] = b;
    }

    g_touch_raw_x = xs[2];
    g_touch_raw_y = ys[2];
    g_touch_raw_z = z;

    if (z < TOUCH_Z_THRESH) return false;
    if (xs[2] < 16 || ys[2] < 16 || xs[2] > 4080 || ys[2] > 4080) return false;

    *rx = xs[2];
    *ry = ys[2];
    return true;
}

static bool touch_read(uint16_t *sx, uint16_t *sy)
{
    uint16_t rx, ry;
    if (!touch_raw(&rx, &ry)) return false;

    // Swap in the RAW domain, before mapping. This is the fix.
    uint16_t ax = g_tcal.swap ? ry : rx;
    uint16_t ay = g_tcal.swap ? rx : ry;

    long mx = map((long)ax, g_tcal.x_min, g_tcal.x_max, 0, SCR_W - 1);
    long my = map((long)ay, g_tcal.y_min, g_tcal.y_max, 0, SCR_H - 1);

    if (g_tcal.inv_x) mx = SCR_W - 1 - mx;
    if (g_tcal.inv_y) my = SCR_H - 1 - my;

    if (mx < 0) mx = 0;
    if (mx > SCR_W - 1) mx = SCR_W - 1;
    if (my < 0) my = 0;
    if (my > SCR_H - 1) my = SCR_H - 1;

    *sx = (uint16_t)mx;
    *sy = (uint16_t)my;
    return true;
}

// Wait for a stable press and return its raw coordinates. Requires the finger
// to be lifted first, so consecutive targets cannot be satisfied by one long
// press, and averages a short burst once pressure is steady.
static bool touch_await_press(uint16_t *rx, uint16_t *ry, uint32_t timeout_ms)
{
    uint32_t t0 = millis();
    uint16_t a, b;

    while (touch_raw(&a, &b)) {                       // wait for release
        if ((uint32_t)(millis() - t0) > timeout_ms) return false;
        delay(20);
    }
    delay(80);

    while (!touch_raw(&a, &b)) {                      // wait for press
        if ((uint32_t)(millis() - t0) > timeout_ms) return false;
        delay(10);
    }

    delay(60);                                        // let pressure settle
    uint32_t sx = 0, sy = 0; int n = 0;
    for (int i = 0; i < 12; i++) {
        if (touch_raw(&a, &b)) { sx += a; sy += b; n++; }
        delay(10);
    }
    if (n < 6) return false;
    *rx = (uint16_t)(sx / n);
    *ry = (uint16_t)(sy / n);
    return true;
}

// ---------------------------------------------------------------------------
// Guided 4-point calibration.
//
// Draws a crosshair at four inset corners and records the raw reading at each.
// From those four samples it works out:
//   * whether the controller axes are transposed, by checking which raw axis
//     varies more between two points separated horizontally on screen
//   * the raw span of each axis, extrapolated from the inset to the full
//     screen so the edges remain reachable
//   * whether each axis runs forwards or backwards
// Nothing is guessed and nothing has to be entered by hand.
// ---------------------------------------------------------------------------
#define TCAL_INSET 28

static bool touch_calibrate_interactive()
{
    const int16_t px[4] = { TCAL_INSET, SCR_W - 1 - TCAL_INSET,
                            SCR_W - 1 - TCAL_INSET, TCAL_INSET };
    const int16_t py[4] = { TCAL_INSET, TCAL_INSET,
                            SCR_H - 1 - TCAL_INSET, SCR_H - 1 - TCAL_INSET };
    uint16_t rx[4], ry[4];

    for (int i = 0; i < 4; i++) {
        fill_screen(C_BLACK);
        draw_text_12(40, 24, "TOUCH CALIBRATION", C_CYAN, C_BLACK);
        draw_text(40, 52, "Tap the centre of each crosshair.", C_WHITE, C_BLACK);
        draw_text(40, 66, "Use a stylus or a fingernail if you can.", C_DIM, C_BLACK);
        char b[32];
        snprintf(b, sizeof(b), "point %d of 4", i + 1);
        draw_text(40, 88, b, C_DIM, C_BLACK);

        // crosshair
        draw_hline(px[i] - 14, py[i], 29, C_AMBER);
        draw_vline(px[i], py[i] - 14, 29, C_AMBER);
        fill_rect(px[i] - 2, py[i] - 2, 5, 5, C_RED);

        if (!touch_await_press(&rx[i], &ry[i], 30000)) {
            fill_screen(C_BLACK);
            draw_text_12(20, 100, "CALIBRATION TIMED OUT", C_RED, C_BLACK);
            draw_text(20, 126, "Using previous values. Retry from SETUP.", C_WHITE, C_BLACK);
            delay(2500);
            return false;
        }
        delay(150);
    }

    // Points 0 and 1 differ in screen X only. Whichever raw axis moved more
    // between them is the axis that tracks screen X.
    long dx_rawx = labs((long)rx[1] - (long)rx[0]);
    long dx_rawy = labs((long)ry[1] - (long)ry[0]);
    bool swap = (dx_rawy > dx_rawx);

    // Collect each point's raw value on the axis that corresponds to screen X
    // and screen Y, applying the swap we just determined.
    long ax[4], ay[4];
    for (int i = 0; i < 4; i++) {
        ax[i] = swap ? ry[i] : rx[i];
        ay[i] = swap ? rx[i] : ry[i];
    }

    // Average the two points at each screen edge.
    long x_left   = (ax[0] + ax[3]) / 2;    // screen x = TCAL_INSET
    long x_right  = (ax[1] + ax[2]) / 2;    // screen x = W-1-TCAL_INSET
    long y_top    = (ay[0] + ay[1]) / 2;    // screen y = TCAL_INSET
    long y_bottom = (ay[2] + ay[3]) / 2;    // screen y = H-1-TCAL_INSET

    bool inv_x = (x_right < x_left);
    bool inv_y = (y_bottom < y_top);
    if (inv_x) { long t = x_left; x_left = x_right; x_right = t; }
    if (inv_y) { long t = y_top;  y_top  = y_bottom; y_bottom = t; }

    // Extrapolate from the inset targets out to the true screen edges, or the
    // outer 28px of the display would be unreachable.
    long span_x = x_right - x_left, span_y = y_bottom - y_top;
    long used_x = (SCR_W - 1) - 2 * TCAL_INSET;
    long used_y = (SCR_H - 1) - 2 * TCAL_INSET;
    if (span_x < 200 || span_y < 200) {
        fill_screen(C_BLACK);
        draw_text_12(16, 96, "CALIBRATION FAILED", C_RED, C_BLACK);
        draw_text(16, 122, "Readings too close together. Tap the", C_WHITE, C_BLACK);
        draw_text(16, 136, "crosshair centres, not the same spot.", C_WHITE, C_BLACK);
        delay(3000);
        return false;
    }
    long per_x = span_x / used_x, per_y = span_y / used_y;

    g_tcal.x_min = (uint16_t)(x_left   - per_x * TCAL_INSET);
    g_tcal.x_max = (uint16_t)(x_right  + per_x * TCAL_INSET);
    g_tcal.y_min = (uint16_t)(y_top    - per_y * TCAL_INSET);
    g_tcal.y_max = (uint16_t)(y_bottom + per_y * TCAL_INSET);
    g_tcal.swap  = swap;
    g_tcal.inv_x = inv_x;
    g_tcal.inv_y = inv_y;
    g_tcal.valid = true;
    touch_cal_save();

    // Immediate verification: a dot must appear under the finger. This is the
    // whole point - the operator confirms the mapping works before relying on
    // it, rather than discovering mid-sweep that buttons do not respond.
    fill_screen(C_BLACK);
    draw_text_12(28, 18, "CHECK THE MAPPING", C_GREEN, C_BLACK);
    draw_text(28, 44, "Drag around. The dot should sit under", C_WHITE, C_BLACK);
    draw_text(28, 58, "your finger, and the corners must work.", C_WHITE, C_BLACK);
    draw_text(28, 80, "Tap the box to accept. Wait 12s to redo.", C_AMBER, C_BLACK);
    draw_rect(110, 180, 100, 34, C_CYAN);
    draw_text(140, 193, "ACCEPT", C_CYAN, C_BLACK);

    uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < 12000) {
        uint16_t tx, ty;
        if (touch_read(&tx, &ty)) {
            fill_rect((int16_t)tx - 2, (int16_t)ty - 2, 5, 5, C_GREEN);
            if (tx >= 110 && tx <= 210 && ty >= 180 && ty <= 214) {
                delay(200);
                return true;
            }
        }
        delay(15);
    }
    return false;    // no accept -> caller offers another attempt
}

// ===========================================================================
// RGB LED  -  tier at a glance without looking at the screen
// ===========================================================================
static void led_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    neopixelWrite(PIN_WS2812, r, g, b);
}

// ===========================================================================
// PROXIMITY GATE
//
// Logic here is byte-identical to the version verified by test_gate.cpp on a
// PC (link budget, hysteresis, the 3-packet rule, BLE reference handling).
// If you change anything in this section, re-run that test.
// ===========================================================================

// (type definitions hoisted to the TYPE DEFINITIONS block near the top -
//  see the note there before moving them back)

// (type definitions hoisted to the TYPE DEFINITIONS block near the top -
//  see the note there before moving them back)

static prox_cal_t g_cal;

static void prox_cal_reset_defaults()
{
    memset(&g_cal, 0, sizeof(g_cal));
    g_cal.near_thresh[BAND_24]     = THRESH_NEAR_24;
    g_cal.contact_thresh[BAND_24]  = THRESH_CONTACT_24;
    g_cal.near_thresh[BAND_5]      = THRESH_NEAR_5;
    g_cal.contact_thresh[BAND_5]   = THRESH_CONTACT_5;
    g_cal.near_thresh[BAND_BLE]    = THRESH_NEAR_BLE;
    g_cal.contact_thresh[BAND_BLE] = THRESH_CONTACT_BLE;
    g_cal.ble_1m_loss              = BLE_1M_LOSS_DB;
    g_cal.pathloss_n               = PATHLOSS_EXPONENT;
}

static void prox_cal_save()
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, NVS_KEY, &g_cal, sizeof(prox_cal_t));
    nvs_commit(h);
    nvs_close(h);
}

static void prox_init()
{
    prox_cal_reset_defaults();
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t len = sizeof(prox_cal_t);
    prox_cal_t st;
    if (nvs_get_blob(h, NVS_KEY, &st, &len) == ESP_OK && len == sizeof(st)) {
        // Sanity-check before trusting a stored blob. A corrupt value that
        // widened the gate would silently turn this into a room-scale scanner,
        // which is the exact failure mode the design exists to avoid.
        bool sane = st.pathloss_n > 1.0f && st.pathloss_n < 4.0f;
        for (int b = 0; b < BAND_COUNT && sane; b++) {
            if (st.near_thresh[b] > -10 || st.near_thresh[b] < -90) sane = false;
            if (st.contact_thresh[b] <= st.near_thresh[b])          sane = false;
        }
        if (sane) g_cal = st;
    }
    nvs_close(h);
}

static void ring_reset(rssi_ring_t *r) { r->head = 0; r->count = 0; }

static void ring_push(rssi_ring_t *r, int8_t rssi, uint32_t now)
{
    r->s[r->head].rssi = rssi;
    r->s[r->head].ts   = now;
    r->head = (uint8_t)((r->head + 1) % RSSI_RING_LEN);
    if (r->count < RSSI_RING_LEN) r->count++;
}

static uint8_t ring_window(const rssi_ring_t *r, uint32_t now, uint32_t win,
                           int8_t *out, uint8_t cap)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < r->count && n < cap; i++) {
        uint8_t idx = (uint8_t)((r->head + RSSI_RING_LEN - 1 - i) % RSSI_RING_LEN);
        if ((uint32_t)(now - r->s[idx].ts) > win) break;   // rollover-safe
        out[n++] = r->s[idx].rssi;
    }
    return n;
}

static int8_t ring_peak(const rssi_ring_t *r, uint32_t now, uint32_t win)
{
    int8_t b[RSSI_RING_LEN];
    uint8_t n = ring_window(r, now, win, b, RSSI_RING_LEN);
    if (!n) return INT8_MIN;
    int8_t pk = b[0];
    for (uint8_t i = 1; i < n; i++) if (b[i] > pk) pk = b[i];
    return pk;
}

static int8_t ring_median(const rssi_ring_t *r, uint32_t now, uint32_t win)
{
    int8_t b[RSSI_RING_LEN];
    uint8_t n = ring_window(r, now, win, b, RSSI_RING_LEN);
    if (!n) return INT8_MIN;
    for (uint8_t i = 1; i < n; i++) {
        int8_t v = b[i]; int8_t j = (int8_t)(i - 1);
        while (j >= 0 && b[j] > v) { b[j+1] = b[j]; j--; }
        b[j+1] = v;
    }
    return b[n / 2];
}

static uint8_t ring_count_above(const rssi_ring_t *r, uint32_t now,
                                uint32_t win, int8_t th)
{
    int8_t b[RSSI_RING_LEN];
    uint8_t n = ring_window(r, now, win, b, RSSI_RING_LEN), c = 0;
    for (uint8_t i = 0; i < n; i++) if (b[i] >= th) c++;
    return c;
}

// --- BLE calibrated references --------------------------------------------
static ranging_ref_t prox_ref_from_txpower(int8_t tx_dbm)
{
    ranging_ref_t r;
    r.src = REF_ADV_TXPOWER;
    r.rssi_at_1m = (int8_t)(tx_dbm - g_cal.ble_1m_loss);
    return r;
}
static ranging_ref_t prox_ref_from_ibeacon(int8_t measured_power)
{
    // iBeacon's measured power IS the calibrated RSSI at 1 m: no
    // implementation-margin guesswork at all, so this is the best reference.
    ranging_ref_t r; r.src = REF_IBEACON; r.rssi_at_1m = measured_power; return r;
}
static ranging_ref_t prox_ref_from_eddystone(int8_t ranging_power)
{
    // Eddystone ranging data is power at 0 m; the spec's own conversion to a
    // 1 m reference is a 41 dB offset.
    ranging_ref_t r; r.src = REF_EDDYSTONE;
    r.rssi_at_1m = (int8_t)(ranging_power - 41); return r;
}

static float rssi_to_distance(int8_t rssi, int8_t at1m, float n)
{ return powf(10.0f, ((float)at1m - (float)rssi) / (10.0f * n)); }

static int8_t distance_to_rssi(float d, int8_t at1m, float n)
{
    if (d <= 0.01f) d = 0.01f;
    return (int8_t)lrintf((float)at1m - 10.0f * n * log10f(d));
}

static float prox_distance_m(const rssi_ring_t *r, const ranging_ref_t *ref,
                             uint32_t now)
{
    if (!ref || ref->src == REF_NONE) return -1.0f;
    int8_t med = ring_median(r, now, MEDIAN_WINDOW_MS);
    if (med == INT8_MIN) return -1.0f;
    return rssi_to_distance(med, ref->rssi_at_1m, g_cal.pathloss_n);
}

static tier_t prox_resolve_tier(const rssi_ring_t *r, rband_t band,
                                const ranging_ref_t *ref, tier_t prev,
                                uint32_t now)
{
    if (band >= BAND_COUNT) return TIER_AMBIENT;

    int8_t near_t, contact_t;
    if (ref && ref->src != REF_NONE) {
        // Derive thresholds from the distance targets, which removes transmit
        // power as an error term. This is the fix for BLE, where source power
        // spans ~28 dB and would otherwise swamp the wall margin entirely.
        near_t    = distance_to_rssi(DIST_NEAR_M,    ref->rssi_at_1m, g_cal.pathloss_n);
        contact_t = distance_to_rssi(DIST_CONTACT_M, ref->rssi_at_1m, g_cal.pathloss_n);
    } else {
        near_t    = g_cal.near_thresh[band];
        contact_t = g_cal.contact_thresh[band];
    }

    // Hysteresis applies only to the tier we are already in.
    if (prev >= TIER_NEAR)    near_t    = (int8_t)(near_t    - TIER_HYSTERESIS_DB);
    if (prev >= TIER_CONTACT) contact_t = (int8_t)(contact_t - TIER_HYSTERESIS_DB);

    // Gate on PEAK. A real 2 m device produces genuinely strong packets; a
    // median gets dragged into the floor by multipath nulls and you would
    // reject things sitting right in front of you.
    int8_t pk = ring_peak(r, now, PEAK_WINDOW_MS);
    if (pk == INT8_MIN) return TIER_AMBIENT;

    if (pk >= contact_t &&
        ring_count_above(r, now, PEAK_WINDOW_MS, contact_t) >= MIN_PKTS_FOR_NEAR)
        return TIER_CONTACT;
    if (pk >= near_t &&
        ring_count_above(r, now, PEAK_WINDOW_MS, near_t) >= MIN_PKTS_FOR_NEAR)
        return TIER_NEAR;
    return TIER_AMBIENT;
}

static const char *tier_name(tier_t t)
{
    return t == TIER_CONTACT ? "CONTACT" : t == TIER_NEAR ? "NEAR" : "ambient";
}
static const char *band_name(rband_t b)
{
    return b == BAND_24 ? "2.4G" : b == BAND_5 ? "5G" : b == BAND_BLE ? "BLE" : "?";
}
static uint16_t tier_colour(tier_t t)
{
    return t == TIER_CONTACT ? C_RED : t == TIER_NEAR ? C_AMBER : C_GREEN;
}

// ===========================================================================
// TRACK TABLE
//
// Ambient-tier observations are deliberately KEPT rather than discarded: the
// corridor/room baseline diff is the real out-of-room filter and it only
// works if sub-threshold history survives.
// ===========================================================================

// (type definitions hoisted to the TYPE DEFINITIONS block near the top -
//  see the note there before moving them back)

// (type definitions hoisted to the TYPE DEFINITIONS block near the top -
//  see the note there before moving them back)

static track_t *g_tracks = NULL;
static uint16_t g_used = 0;
static phase_t  g_phase = PHASE_FREE;
// Concurrency: the pump task (priority 5) writes tracks while the UI task
// reads them. There is deliberately NO lock. Nothing here can crash on a race
// because the table is a fixed PSRAM array that is never freed, so a torn
// read of a counter or a float costs at most one garbled pixel row for 220 ms.
// Taking a spinlock in the capture hot path to buy that back would cost real
// frames, and dropped frames lose evidence. The one race that DID matter -
// the detail screen holding a pointer to a slot that got evicted and reused
// for a different device - is fixed by keying the selection on MAC and band
// and re-resolving it on every draw. See s_sel_mac below.

static void classify_track(track_t *t);        // forward
static void triage_update(track_t *t, uint32_t now);   // forward

static void tracks_reset()
{
    if (!g_tracks) return;
    memset(g_tracks, 0, (size_t)MAX_TRACKS * sizeof(track_t));
    g_used = 0;
    g_phase = PHASE_FREE;
}

static bool tracks_init()
{
    if (g_tracks) return true;
    // PSRAM: 600 tracks is roughly 250 kB, uncomfortable in internal SRAM but
    // nothing against 8 MB. Requires PSRAM: Enabled in the Tools menu.
    g_tracks = (track_t *)heap_caps_calloc(MAX_TRACKS, sizeof(track_t),
                                           MALLOC_CAP_SPIRAM);
    if (!g_tracks) {
        Serial.println("[tracks] PSRAM alloc failed - is PSRAM enabled in Tools?");
        g_tracks = (track_t *)calloc(MAX_TRACKS, sizeof(track_t));
    }
    if (!g_tracks) { Serial.println("[tracks] FATAL: no memory"); return false; }
    tracks_reset();
    return true;
}

static uint16_t mac_hash(const uint8_t mac[6], rband_t band)
{
    uint32_t h = 2166136261u;
    for (int i = 0; i < 6; i++) { h ^= mac[i]; h *= 16777619u; }
    h ^= (uint32_t)band; h *= 16777619u;
    return (uint16_t)(h % MAX_TRACKS);
}

// A device heard on both bands gets separate tracks on purpose: the
// thresholds differ per band and mixing them would corrupt the gate.
static track_t *track_get(const uint8_t mac[6], rband_t band, bool create)
{
    if (!g_tracks) return NULL;
    uint16_t idx = mac_hash(mac, band);
    int16_t  freeslot = -1;

    for (uint16_t p = 0; p < MAX_TRACKS; p++) {
        uint16_t i = (uint16_t)((idx + p) % MAX_TRACKS);
        track_t *t = &g_tracks[i];
        if (!t->used) { freeslot = (int16_t)i; break; }
        if (t->band == band && memcmp(t->mac, mac, 6) == 0) return t;
    }
    if (!create) return NULL;

    if (freeslot < 0) {
        // Table full: evict the stalest ambient track. Never evict anything
        // that has ever reached NEAR - that is the operator's evidence.
        uint32_t oldest = UINT32_MAX; int16_t victim = -1;
        for (uint16_t i = 0; i < MAX_TRACKS; i++) {
            if (g_tracks[i].tier_best >= TIER_NEAR) continue;
            if (g_tracks[i].last_ms < oldest) { oldest = g_tracks[i].last_ms; victim = (int16_t)i; }
        }
        if (victim < 0) return NULL;
        freeslot = victim;
        if (g_used) g_used--;
    }

    track_t *t = &g_tracks[freeslot];
    memset(t, 0, sizeof(track_t));
    t->used = true;
    memcpy(t->mac, mac, 6);
    t->band = band;
    t->peak_session = INT8_MIN;
    t->dist_m = -1.0f;
    t->ref.src = REF_NONE;
    ring_reset(&t->ring);
    for (int p = 0; p < PHASE_COUNT; p++) t->peak_phase[p] = INT8_MIN;
    g_used++;
    return t;
}

// RSSI ATTRIBUTION RULE: call this only for the actual transmitter. For WiFi
// that means addr2 and nothing else. Crediting RSSI to addr1 would let a
// distant camera inherit the signal strength of a nearby AP talking to it,
// which would silently destroy the gate.
static void track_observe_rssi(track_t *t, int8_t rssi, uint32_t now)
{
    if (!t) return;
    if (t->first_ms == 0) t->first_ms = now;
    t->last_ms = now;
    t->pkts++;
    ring_push(&t->ring, rssi, now);
    if (rssi > t->peak_session) t->peak_session = rssi;

    if (g_phase != PHASE_FREE && rssi > t->peak_phase[g_phase]) {
        t->peak_phase[g_phase] = rssi;
        t->phase_mask |= (uint8_t)(1u << g_phase);
    }

    if (t->rate_win_start == 0 ||
        (uint32_t)(now - t->rate_win_start) >= STREAM_WINDOW_MS) {
        uint32_t el = now - t->rate_win_start;
        if (t->rate_win_start && el) t->pkt_rate = (t->rate_win_pkts * 1000u) / el;
        t->rate_win_start = now;
        t->rate_win_pkts = 0;
    }
    t->rate_win_pkts++;
}

static void tracks_tick(uint32_t now)
{
    if (!g_tracks) return;
    for (uint16_t i = 0; i < MAX_TRACKS; i++) {
        track_t *t = &g_tracks[i];
        if (!t->used) continue;
        const ranging_ref_t *ref = (t->ref.src != REF_NONE) ? &t->ref : NULL;

        tier_t prev = t->tier;
        t->tier   = prox_resolve_tier(&t->ring, t->band, ref, prev, now);
        t->dist_m = prox_distance_m(&t->ring, ref, now);

        if (t->tier > t->tier_best) t->tier_best = t->tier;
        if (t->tier == TIER_CONTACT) t->evidence |= E_CONTACT_TIER;
        // Alert once per crossing up into NEAR, never repeatedly while a
        // device sits in range - repeated alerts train you to ignore the tool.
        if (t->tier >= TIER_NEAR && prev < TIER_NEAR) t->alerted = false;

        classify_track(t);
        triage_update(t, now);
    }
}

// Sweep peak shape. A device in the room shows a broad elevated region. One
// on the far side of a wall peaks only when you are against that wall. A
// device concealed in an in-room fixture also peaks sharply, but HIGHER -
// into the contact tier - which is how those two sharp cases are told apart.
static void tracks_close_position(uint32_t now)
{
    if (!g_tracks) return;
    for (uint16_t i = 0; i < MAX_TRACKS; i++) {
        track_t *t = &g_tracks[i];
        if (!t->used) continue;

        t->profile[t->profile_head] = ring_peak(&t->ring, now, PEAK_WINDOW_MS * 4);
        t->profile_head = (uint8_t)((t->profile_head + 1) % SWEEP_PROFILE_SLOTS);
        if (t->profile_count < SWEEP_PROFILE_SLOTS) t->profile_count++;
        if (t->profile_count < 4) continue;

        int8_t best = INT8_MIN;
        for (uint8_t k = 0; k < t->profile_count; k++)
            if (t->profile[k] > best) best = t->profile[k];
        if (best == INT8_MIN) continue;

        uint8_t heard = 0, near_best = 0;
        for (uint8_t k = 0; k < t->profile_count; k++) {
            if (t->profile[k] == INT8_MIN) continue;
            heard++;
            if (t->profile[k] >= best - 8) near_best++;
        }
        if (!heard) continue;

        t->evidence &= ~(uint32_t)(E_SHARP_PEAK | E_BROAD_PEAK);
        float frac = (float)near_best / (float)heard;
        if (frac <= 0.34f)      t->evidence |= E_SHARP_PEAK;
        else if (frac >= 0.60f) t->evidence |= E_BROAD_PEAK;
    }
}

static void baseline_apply_diff()
{
    if (!g_tracks) return;
    for (uint16_t i = 0; i < MAX_TRACKS; i++) {
        track_t *t = &g_tracks[i];
        if (!t->used) continue;
        bool in_room = (t->phase_mask & (1u << PHASE_ROOM)) != 0;
        bool in_cor  = (t->phase_mask & (1u << PHASE_CORRIDOR)) != 0;
        if (!in_room) continue;
        if (!in_cor) t->evidence |= E_ROOM_ONLY;      // strongest evidence here
        else if (t->peak_phase[PHASE_ROOM] - t->peak_phase[PHASE_CORRIDOR]
                 >= ROOM_ONLY_DELTA_DB) t->evidence |= E_ROOM_ONLY;
    }
}

// Baseline phase control. The corridor/room diff is the real out-of-room
// filter, and it needs a known dwell in each phase - hence the timestamp, so
// the sweep screen can show a countdown instead of leaving the operator
// guessing when 60 seconds are up.
static uint32_t g_phase_start = 0;

static phase_t baseline_phase(void) { return g_phase; }

static void baseline_set_phase(phase_t p, uint32_t now)
{
    g_phase = p;
    g_phase_start = now;
}

static uint32_t baseline_elapsed_ms(uint32_t now)
{
    return g_phase_start ? (now - g_phase_start) : 0;
}

// Highest risk first - that is the whole point of the list. Ties break on
// the gate tier, then the score, then raw signal, so ordering is fully
// determined and rows do not swap places on equal footing.
static int cmp_tracks(const void *pa, const void *pb)
{
    const track_t *a = *(const track_t **)pa, *b = *(const track_t **)pb;
    if (a->risk != b->risk)           return (int)b->risk - (int)a->risk;
    if (a->tier_best != b->tier_best) return (int)b->tier_best - (int)a->tier_best;
    if (a->score != b->score)         return (int)b->score - (int)a->score;
    if (a->peak_session != b->peak_session)
                                      return (int)b->peak_session - (int)a->peak_session;
    return memcmp(a->mac, b->mac, 6);
}

static uint16_t tracks_sorted(track_t **out, uint16_t cap)
{
    if (!g_tracks) return 0;
    uint16_t n = 0;
    for (uint16_t i = 0; i < MAX_TRACKS && n < cap; i++)
        if (g_tracks[i].used) out[n++] = &g_tracks[i];
    qsort(out, n, sizeof(track_t *), cmp_tracks);
    return n;
}

// ===========================================================================
// SIGNATURES  -  the field-editable part
//
// PROVENANCE WARNING. Read before trusting any match.
//
// Hidden cameras are usually made with no brand at all, or under random names
// absent from the IEEE OUI registry. So OUI matching has poor coverage on
// exactly the devices this tool is for, and the SoC-vendor prefixes that DO
// appear (Realtek, Anyka, Ingenic, Espressif) are shared with thousands of
// innocent devices - including this board itself.
//
// Entries are therefore weighted by how much they actually prove:
//   STRONG  a vendor that only makes cameras, or a product-specific name
//   WEAK    a camera-common SoC vendor or generic keyword. Logged, scored
//           low, never alerts on its own.
//
// The real discriminators are NOT in this table. They are the proximity gate,
// the corridor/room baseline diff and the streaming-traffic heuristic - none
// of which need a database, and all of which survive MAC randomisation.
//
// Treat everything here as a lead requiring field confirmation.
// ===========================================================================

// (type definitions hoisted to the TYPE DEFINITIONS block near the top -
//  see the note there before moving them back)

// WiFi SoftAP SSIDs. Cheap cameras in unprovisioned or standalone mode raise
// an AP with a factory SSID and beacon constantly - the easy catch.
static const str_sig_t SIG_SSID[] = {
    {"ipcam",     SIG_STRONG}, {"ipc-",       SIG_STRONG},
    {"v380",      SIG_STRONG}, {"yicam",      SIG_STRONG},
    {"goolink",   SIG_STRONG}, {"anycam",     SIG_STRONG},
    {"netcam",    SIG_STRONG}, {"wificam",    SIG_STRONG},
    {"clearview", SIG_STRONG}, {"aicam",      SIG_STRONG},
    {"smartcam",  SIG_STRONG}, {"cloudcam",   SIG_STRONG},
    {"lookcam",   SIG_STRONG}, {"littlestar", SIG_STRONG},
    // generic, high false-positive rate - log only
    {"camera",    SIG_WEAK},   {"cam-",       SIG_WEAK},
    {"dvr",       SIG_WEAK},   {"hd-",        SIG_WEAK},
    {"mini",      SIG_WEAK},
};
#define N_SIG_SSID (sizeof(SIG_SSID)/sizeof(SIG_SSID[0]))

static const str_sig_t SIG_BLE_CAM[] = {
    {"gopro",     SIG_STRONG}, {"insta360",  SIG_STRONG},
    {"sjcam",     SIG_STRONG}, {"akaso",     SIG_STRONG},
    {"camremote", SIG_STRONG}, {"spycam",    SIG_STRONG},
    {"bodycam",   SIG_STRONG}, {"dashcam",   SIG_WEAK},
};
#define N_SIG_BLE_CAM (sizeof(SIG_BLE_CAM)/sizeof(SIG_BLE_CAM[0]))

// Wireless mic systems. In a hotel room these are arguably a likelier audio
// vector than a purpose-built bug: cheap, sold openly, long battery life.
static const str_sig_t SIG_BLE_MIC[] = {
    {"dji mic",      SIG_STRONG}, {"wireless go", SIG_STRONG},
    {"rode",         SIG_STRONG}, {"hollyland",   SIG_STRONG},
    {"lark",         SIG_STRONG}, {"boya",        SIG_STRONG},
    {"comica",       SIG_STRONG}, {"saramonic",   SIG_STRONG},
    {"wireless mic", SIG_STRONG}, {"mic-",        SIG_WEAK},
};
#define N_SIG_BLE_MIC (sizeof(SIG_BLE_MIC)/sizeof(SIG_BLE_MIC[0]))

static const oui_sig_t SIG_OUI[] = {
    // surveillance vendors
    {{0x00,0x40,0x48}, SIG_STRONG, "Hikvision"},
    {{0x44,0x19,0xB6}, SIG_STRONG, "Hikvision"},
    {{0xC0,0x56,0xE3}, SIG_STRONG, "Hikvision"},
    {{0x4C,0x11,0xBF}, SIG_STRONG, "Dahua"},
    {{0x90,0x02,0xA9}, SIG_STRONG, "Dahua"},
    {{0x3C,0xEF,0x8C}, SIG_STRONG, "Dahua"},
    {{0xAC,0xCC,0x8E}, SIG_STRONG, "Axis"},
    {{0x00,0x40,0x8C}, SIG_STRONG, "Axis"},
    {{0xE0,0x50,0x8B}, SIG_STRONG, "Uniview"},
    {{0x48,0xEA,0x63}, SIG_STRONG, "Uniview"},
    {{0x00,0x0F,0x7C}, SIG_STRONG, "ACTi"},
    {{0x00,0x02,0xD1}, SIG_STRONG, "Vivotek"},
    {{0x00,0x1A,0x07}, SIG_STRONG, "Arecont"},
    // SoC/module vendors: shared with masses of unrelated IoT, hence WEAK
    {{0x00,0xE0,0x4C}, SIG_WEAK,   "Realtek"},
    {{0x1C,0xBF,0xCE}, SIG_WEAK,   "Shenzhen OEM"},
    {{0x18,0xFE,0x34}, SIG_WEAK,   "Espressif"},
    {{0x7C,0xDF,0xA1}, SIG_WEAK,   "Espressif"},
    {{0x8C,0xCE,0x4E}, SIG_WEAK,   "Anyka cam SoC"},
    {{0x00,0x25,0x9E}, SIG_WEAK,   "Ingenic cam SoC"},
};
#define N_SIG_OUI (sizeof(SIG_OUI)/sizeof(SIG_OUI[0]))

static const uint16_t SIG_SVC16[] = { 0xFEAA, 0xFE59, 0xFDF0 };
#define N_SIG_SVC16 (sizeof(SIG_SVC16)/sizeof(SIG_SVC16[0]))
static const uint16_t SIG_COMPANY[] = { 0x0B84, 0x09C8 };
#define N_SIG_COMPANY (sizeof(SIG_COMPANY)/sizeof(SIG_COMPANY[0]))

static bool istrstr(const char *hay, const char *needle)
{
    if (!hay || !needle || !*needle) return false;
    size_t nl = strlen(needle);
    for (const char *p = hay; *p; p++) {
        size_t i = 0;
        while (i < nl && p[i] &&
               tolower((unsigned char)p[i]) == tolower((unsigned char)needle[i])) i++;
        if (i == nl) return true;
    }
    return false;
}

// Scan the whole table rather than returning on first hit, so a STRONG match
// always wins over an incidental WEAK keyword in the same name.
static bool sig_match_str(const str_sig_t *tbl, uint32_t n, const char *hay,
                          sig_weight_t *w_out)
{
    if (!hay || !hay[0]) return false;
    bool found = false; sig_weight_t best = SIG_WEAK;
    for (uint32_t i = 0; i < n; i++) {
        if (!istrstr(hay, tbl[i].pat)) continue;
        if (!found || tbl[i].w > best) best = tbl[i].w;
        found = true;
        if (best == SIG_STRONG) break;
    }
    if (found && w_out) *w_out = best;
    return found;
}

// sig_match_oui() was replaced by vendor_lookup() in the DEVICE TYPE AND
// VENDOR section below, which returns the vendor string and a device-type
// hint from one table instead of just a weight.

// ===========================================================================
// DEVICE TYPE AND VENDOR
//
// Device type comes from four tiers of evidence, and the tier is carried
// alongside the type because it decides how much the triage stage trusts it:
//
//   TIER A  self-declared in a standards field. The device says what it is,
//           in cleartext, in its own beacon or advertisement:
//             - WPS Primary Device Type in a Wi-Fi beacon
//             - BLE Appearance, or a SIG audio/microphone service UUID
//           This is the strongest signal in the whole firmware.
//   TIER B  behaviour. Sustained uplink-dominant traffic = camera-like.
//           Needs no database and survives MAC randomisation.
//   TIER C  advertised name or SSID matches a pattern.
//   TIER D  the OUI belongs to a vendor that makes cameras. Weakest: plenty
//           of no-name cameras use SoC-vendor prefixes shared with masses of
//           innocent hardware.
//
// VENDOR STRINGS come from two places, and the good one is not the database.
// A Wi-Fi device in SoftAP mode usually broadcasts WPS Manufacturer (0x1021)
// and Model Name (0x1023) attributes as plain strings in its beacon. That is
// exact, needs no lookup, and beats an OUI table outright. The curated OUI
// table below is only the fallback for devices that publish nothing.
// ===========================================================================

static const char *dtype_name(dtype_t d)
{
    switch (d) {
        case DTYPE_CAMERA:   return "CAMERA";
        case DTYPE_MIC:      return "MIC";
        case DTYPE_AV:       return "AV GEAR";
        case DTYPE_NVR:      return "NVR/DVR";
        case DTYPE_PHONE:    return "PHONE";
        case DTYPE_COMPUTER: return "COMPUTER";
        case DTYPE_NETWORK:  return "NETWORK";
        case DTYPE_DISPLAY:  return "DISPLAY";
        case DTYPE_SPEAKER:  return "SPEAKER";
        case DTYPE_WEARABLE: return "WEARABLE";
        case DTYPE_PRINTER:  return "PRINTER";
        case DTYPE_TAG:      return "TAG";
        default:             return "UNKNOWN";
    }
}

static const char *dtier_name(uint8_t t)
{
    switch (t) {
        case DT_TIER_A: return "declared";
        case DT_TIER_B: return "behaviour";
        case DT_TIER_C: return "name";
        case DT_TIER_D: return "vendor";
        default:        return "none";
    }
}

// A camera or a microphone is what this tool exists to find. Everything else
// is context, however interesting.
static bool dtype_is_recorder(dtype_t d)
{
    return d == DTYPE_CAMERA || d == DTYPE_MIC || d == DTYPE_NVR;
}

// ---------------------------------------------------------------------------
// CURATED OUI TABLE
//
// PROVENANCE. Every entry below is hand-checked. It is deliberately SHORT
// rather than padded out to a few hundred guesses: a wrong prefix does not
// fail quietly, it puts a confident vendor name and device type next to
// somebody's phone, and a triage label that is wrong once is worse than no
// label at all.
//
// To extend it accurately, run tools/make_vendor_table.py against a freshly
// downloaded IEEE registry. It filters the real registry by vendor keyword
// and emits entries in this exact format, so the table stays correct and
// current without anyone typing prefixes from memory.
//
// dtype here is a HINT ONLY and is recorded as TIER D. It never sets the
// device type when a WPS or BLE declaration is available.
// ---------------------------------------------------------------------------
static const vendor_sig_t VENDOR_OUI[] = {
    // --- surveillance vendors: these make cameras and NVRs, little else ---
    { {0x00,0x40,0x48}, "Hikvision",      DTYPE_CAMERA,  SIG_STRONG },
    { {0x44,0x19,0xB6}, "Hikvision",      DTYPE_CAMERA,  SIG_STRONG },
    { {0xC0,0x56,0xE3}, "Hikvision",      DTYPE_CAMERA,  SIG_STRONG },
    { {0x4C,0x11,0xBF}, "Dahua",          DTYPE_CAMERA,  SIG_STRONG },
    { {0x90,0x02,0xA9}, "Dahua",          DTYPE_CAMERA,  SIG_STRONG },
    { {0x3C,0xEF,0x8C}, "Dahua",          DTYPE_CAMERA,  SIG_STRONG },
    { {0xAC,0xCC,0x8E}, "Axis",           DTYPE_CAMERA,  SIG_STRONG },
    { {0x00,0x40,0x8C}, "Axis",           DTYPE_CAMERA,  SIG_STRONG },
    { {0xE0,0x50,0x8B}, "Uniview",        DTYPE_CAMERA,  SIG_STRONG },
    { {0x48,0xEA,0x63}, "Uniview",        DTYPE_CAMERA,  SIG_STRONG },
    { {0x00,0x0F,0x7C}, "ACTi",           DTYPE_CAMERA,  SIG_STRONG },
    { {0x00,0x02,0xD1}, "Vivotek",        DTYPE_CAMERA,  SIG_STRONG },
    { {0x00,0x1A,0x07}, "Arecont Vision", DTYPE_CAMERA,  SIG_STRONG },

    // --- SoC and module vendors found inside no-name cameras.
    //     Shared with thousands of unrelated devices, so WEAK and typed
    //     UNKNOWN: presence is a lead, not an identification. ---
    { {0x8C,0xCE,0x4E}, "Anyka",          DTYPE_UNKNOWN, SIG_WEAK },
    { {0x00,0x25,0x9E}, "Ingenic",        DTYPE_UNKNOWN, SIG_WEAK },
    { {0x00,0xE0,0x4C}, "Realtek",        DTYPE_UNKNOWN, SIG_WEAK },
    { {0x1C,0xBF,0xCE}, "Shenzhen OEM",   DTYPE_UNKNOWN, SIG_WEAK },
    { {0x18,0xFE,0x34}, "Espressif",      DTYPE_UNKNOWN, SIG_WEAK },
    { {0x7C,0xDF,0xA1}, "Espressif",      DTYPE_UNKNOWN, SIG_WEAK },
};
#define N_VENDOR_OUI (sizeof(VENDOR_OUI)/sizeof(VENDOR_OUI[0]))

static const vendor_sig_t *vendor_lookup(const uint8_t mac[6])
{
    // A locally-administered address carries no vendor information at all
    // (bit 1 of octet 0), and plenty of cameras randomise. Matching one
    // against a vendor table produces confident nonsense, so skip it.
    if (mac[0] & 0x02) return NULL;
    for (uint32_t i = 0; i < N_VENDOR_OUI; i++)
        if (memcmp(mac, VENDOR_OUI[i].oui, 3) == 0) return &VENDOR_OUI[i];
    return NULL;
}

// Record a device type only if the new evidence is at least as strong as what
// we already had. Stops a name keyword from overwriting a WPS declaration.
static void dtype_claim(track_t *t, dtype_t d, uint8_t tier)
{
    if (d == DTYPE_UNKNOWN) return;
    if (t->dtype != DTYPE_UNKNOWN && t->dtype_tier <= tier) return;  // A < D
    t->dtype = d;
    t->dtype_tier = tier;
}

// ---------------------------------------------------------------------------
// WPS Primary Device Type -> our device type.
//
// The attribute value is 8 bytes: 2-byte CategoryID, 4-byte OUI
// (00 50 F2 04 for the predefined categories), 2-byte SubcategoryID.
// Camera is category 4. WSC 1.0 defined only subcategory 1, Digital Still
// Camera; WSC 2.0 adds video, web and security camera, and an Audio Devices
// category. The subcategory numbers below are best-effort and should be
// checked against a live capture - but the CATEGORY is what drives typing
// here, so an unrecognised subcategory still yields the right answer.
// ---------------------------------------------------------------------------
#define WPS_CAT_COMPUTER   1
#define WPS_CAT_INPUT      2
#define WPS_CAT_PRINTER    3
#define WPS_CAT_CAMERA     4
#define WPS_CAT_STORAGE    5
#define WPS_CAT_NETWORK    6
#define WPS_CAT_DISPLAY    7
#define WPS_CAT_MULTIMEDIA 8
#define WPS_CAT_GAMING     9
#define WPS_CAT_TELEPHONE 10
#define WPS_CAT_AUDIO     11

static dtype_t wps_category_to_dtype(uint16_t cat, uint16_t sub, const char **detail)
{
    *detail = NULL;
    switch (cat) {
        case WPS_CAT_CAMERA:
            switch (sub) {
                case 1: *detail = "still camera";    break;
                case 2: *detail = "video camera";    break;
                case 3: *detail = "web camera";      break;
                case 4: *detail = "security camera"; break;
                default: *detail = "camera";         break;
            }
            return DTYPE_CAMERA;
        case WPS_CAT_AUDIO:      *detail = "audio device"; return DTYPE_MIC;
        case WPS_CAT_MULTIMEDIA: *detail = "multimedia";   return DTYPE_AV;
        case WPS_CAT_DISPLAY:    return DTYPE_DISPLAY;
        case WPS_CAT_NETWORK:    return DTYPE_NETWORK;
        case WPS_CAT_COMPUTER:   return DTYPE_COMPUTER;
        case WPS_CAT_TELEPHONE:  return DTYPE_PHONE;
        case WPS_CAT_PRINTER:    return DTYPE_PRINTER;
        default:                 return DTYPE_UNKNOWN;
    }
}

// ---------------------------------------------------------------------------
// BLE Appearance -> our device type.
//
// Appearance is 16 bits: bits 15..6 category, bits 5..0 subcategory. The
// categories below are from the SIG assigned numbers; verify against a
// capture before relying on the exact values. Audio Source is the notable
// one - a wireless microphone is an audio SOURCE, while a speaker is a sink,
// so the two must not be conflated.
// ---------------------------------------------------------------------------
#define APPEAR_PHONE        0x01
#define APPEAR_COMPUTER     0x02
#define APPEAR_WATCH        0x03
#define APPEAR_DISPLAY      0x05
#define APPEAR_REMOTE       0x06
#define APPEAR_TAG          0x08
#define APPEAR_MEDIA_PLAYER 0x0A
#define APPEAR_AUDIO_SINK   0x21
#define APPEAR_AUDIO_SOURCE 0x22
#define APPEAR_WEARABLE_AUD 0x25
#define APPEAR_AV_EQUIPMENT 0x27
#define APPEAR_DISPLAY_EQ   0x28

static dtype_t appearance_to_dtype(uint16_t appear)
{
    switch (appear >> 6) {
        case APPEAR_AUDIO_SOURCE: return DTYPE_MIC;      // mic, not speaker
        case APPEAR_WEARABLE_AUD: return DTYPE_MIC;
        case APPEAR_AV_EQUIPMENT: return DTYPE_AV;
        case APPEAR_MEDIA_PLAYER: return DTYPE_AV;
        case APPEAR_AUDIO_SINK:   return DTYPE_SPEAKER;
        case APPEAR_PHONE:        return DTYPE_PHONE;
        case APPEAR_COMPUTER:     return DTYPE_COMPUTER;
        case APPEAR_WATCH:        return DTYPE_WEARABLE;
        case APPEAR_TAG:          return DTYPE_TAG;
        case APPEAR_DISPLAY:
        case APPEAR_DISPLAY_EQ:   return DTYPE_DISPLAY;
        case APPEAR_REMOTE:       return DTYPE_UNKNOWN;
        default:                  return DTYPE_UNKNOWN;
    }
}

// SIG service UUIDs that imply audio capture. 0x184D Microphone Control and
// 0x1843 Audio Input Control are the two that actually mean "this thing has a
// microphone in it". Verify against a capture; treat as TIER A when present.
#define UUID_MICROPHONE_CONTROL 0x184D
#define UUID_AUDIO_INPUT_CTRL   0x1843

static dtype_t svc_uuid_to_dtype(uint16_t u)
{
    if (u == UUID_MICROPHONE_CONTROL || u == UUID_AUDIO_INPUT_CTRL) return DTYPE_MIC;
    return DTYPE_UNKNOWN;
}

// ===========================================================================
// TRIAGE
//
// Risk is two independent questions, and collapsing them into one number
// loses the distinction that matters:
//
//   1. how confident are we this is a recording device?   (dtype + tier)
//   2. how confident are we it is in THIS room?           (gate tier,
//                                                          baseline diff)
//
// The list has to be ordered, so the two are combined into a level - but the
// rule that fired is recorded and shown on the detail screen, so the label is
// auditable rather than an opaque score. A triage label is a claim; a
// CRITICAL on a soundbar costs the operator's trust permanently, and after
// that they ignore the tool.
//
// Policy: BALANCED. Strong type evidence at NEAR is enough for CRITICAL -
// contact range or a baseline diff is not required. The trade is accepted
// deliberately: in a hotel room a declared camera two metres away deserves
// the top of the list even if the operator never ran a corridor baseline.
// ===========================================================================

static const char *risk_name(risk_t r)
{
    switch (r) {
        case RISK_CRITICAL: return "CRIT";
        case RISK_HIGH:     return "HIGH";
        case RISK_MEDIUM:   return "MED";
        default:            return "LOW";
    }
}

static uint16_t risk_colour(risk_t r)
{
    switch (r) {
        case RISK_CRITICAL: return C_RED;
        case RISK_HIGH:     return C_AMBER;
        case RISK_MEDIUM:   return C_YELLOW;
        default:            return C_PANEL;
    }
}

// Evaluate the rule table. First match wins, and each rule carries the reason
// string that the detail screen shows.
static risk_t triage_eval(const track_t *t, const char **why)
{
    bool recorder   = dtype_is_recorder(t->dtype);
    bool hard_type  = recorder && t->dtype_tier <= DT_TIER_B;  // declared or behaviour
    bool soft_type  = recorder && t->dtype_tier >  DT_TIER_B;  // name or vendor only
    bool contact    = (t->tier_best >= TIER_CONTACT);
    bool near       = (t->tier_best >= TIER_NEAR);
    bool room_only  = (t->evidence & E_ROOM_ONLY) != 0;
    bool streaming  = (t->evidence & E_STREAM_PROFILE) != 0;

    // ---- CRITICAL ----
    if (contact && recorder) {
        *why = "recorder at contact range";
        return RISK_CRITICAL;
    }
    if (near && hard_type) {
        // The balanced policy in one line.
        *why = (t->dtype_tier == DT_TIER_A) ? "declared recorder within gate"
                                            : "recording behaviour within gate";
        return RISK_CRITICAL;
    }
    if (near && room_only && recorder) {
        *why = "recorder, in-room only vs corridor";
        return RISK_CRITICAL;
    }

    // ---- HIGH ----
    if (near && soft_type) {
        *why = "possible recorder within gate";
        return RISK_HIGH;
    }
    if (contact && streaming) {
        *why = "streaming device at contact range";
        return RISK_HIGH;
    }
    if (near && room_only) {
        *why = "in-room only vs corridor baseline";
        return RISK_HIGH;
    }
    if (contact) {
        *why = "unidentified device at contact range";
        return RISK_HIGH;
    }

    // ---- MEDIUM ----
    if (near) {
        *why = t->evidence ? "within gate, some evidence" : "within gate";
        return RISK_MEDIUM;
    }
    if (hard_type) {
        // Out of the gate but the device declares itself a recorder. Capped
        // at MEDIUM on purpose: a perfect match in the next room is not this
        // operator's problem, and letting it climb would crowd out the thing
        // actually in the smoke detector.
        *why = "recorder, but outside the gate";
        return RISK_MEDIUM;
    }

    *why = "no proximity, no type evidence";
    return RISK_LOW;
}

// Level hysteresis. Without this the badge flips between levels on multipath
// exactly the way the RSSI tier used to, and a list whose labels churn is
// unreadable. A level must be sustained before it is shown, and DEMOTION is
// slower than promotion so a device does not quietly drop off the top of the
// list between glances.
#define RISK_PROMOTE_MS  1500
#define RISK_DEMOTE_MS   6000

static void triage_update(track_t *t, uint32_t now)
{
    const char *why = NULL;
    risk_t raw = triage_eval(t, &why);
    t->risk_why = why;

    if (raw == t->risk) { t->risk_since = now; return; }

    if (t->risk_since == 0) { t->risk_since = now; t->risk_raw = raw; }
    if (raw != t->risk_raw) { t->risk_raw = raw; t->risk_since = now; return; }

    uint32_t need = (raw > t->risk) ? RISK_PROMOTE_MS : RISK_DEMOTE_MS;
    if ((uint32_t)(now - t->risk_since) >= need) {
        t->risk = raw;
        t->risk_since = now;
    }
}

// ===========================================================================
// CLASSIFIER
//
// Two rules that matter:
//  1. Scoring runs on every track, but ALERTS only fire on NEAR or better.
//     A perfect signature match 20 m away in the next room is not this tool's
//     problem, and surfacing it is how a sweep tool becomes unreadable.
//  2. No single field reaches the HIGH band alone. Confidence comes from a
//     proximity tier plus independent corroboration.
// ===========================================================================

// (type definitions hoisted to the TYPE DEFINITIONS block near the top -
//  see the note there before moving them back)

static int score_for(uint32_t bit)
{
    switch (bit) {
        case E_SOFTAP_CAM_SSID: return 40;
        case E_CAM_OUI_STRONG:  return 25;
        case E_CAM_OUI_WEAK:    return  5;
        case E_STREAM_PROFILE:  return 40;
        case E_CONTACT_TIER:    return 25;
        case E_ROOM_ONLY:       return 30;
        case E_WILDCARD_PROBE:  return 10;
        case E_HIDDEN_SSID:     return 10;
        case E_BLE_CAM_NAME:    return 35;
        case E_BLE_MIC_NAME:    return 35;
        case E_BLE_SVC_UUID:    return 20;
        case E_BLE_COMPANY:     return 10;
        case E_SHARP_PEAK:      return  0;   // shape alone is not evidence
        case E_BROAD_PEAK:      return 10;   // breadth weakly favours in-room
        default:                return  0;
    }
}

// The streaming heuristic. Strongest signal available for a camera that has
// joined the room's own WiFi, where there are no beacons to catch and the MAC
// may be randomised. Excludes APs deliberately: an access point is
// uplink-dominant from our vantage point too, and the hotel's own AP is not
// what we are hunting.
static bool looks_like_streaming(const track_t *t)
{
    if (t->is_ap) return false;
    if (t->bytes_up < STREAM_MIN_BYTES_TX) return false;
    if (t->pkt_rate < STREAM_MIN_PKT_RATE) return false;
    float ratio = (t->bytes_down > 0)
                ? (float)t->bytes_up / (float)t->bytes_down
                : STREAM_MIN_UPLINK_RATIO + 1.0f;
    return ratio >= STREAM_MIN_UPLINK_RATIO;
}

static void classify_track(track_t *t)
{
    if (!t) return;
    sig_weight_t w;

    if (t->band == BAND_24 || t->band == BAND_5) {
        if (t->name[0] && sig_match_str(SIG_SSID, N_SIG_SSID, t->name, &w)) {
            t->evidence |= (w == SIG_STRONG) ? E_SOFTAP_CAM_SSID : E_CAM_OUI_WEAK;
            if (w == SIG_STRONG) dtype_claim(t, DTYPE_CAMERA, DT_TIER_C);
        }
        if (looks_like_streaming(t)) {
            t->evidence |= E_STREAM_PROFILE;
            // Sustained uplink-dominant traffic from a non-AP is camera-like
            // behaviour. TIER B: no database, survives MAC randomisation.
            dtype_claim(t, DTYPE_CAMERA, DT_TIER_B);
        } else {
            t->evidence &= ~(uint32_t)E_STREAM_PROFILE;
        }
    }

    // Curated OUI table: vendor string as a fallback, device type only at
    // TIER D so it never overrides a declaration.
    const vendor_sig_t *v = vendor_lookup(t->mac);
    if (v) {
        t->evidence |= (v->w == SIG_STRONG) ? E_CAM_OUI_STRONG : E_CAM_OUI_WEAK;
        if (!t->vendor[0]) {
            strncpy(t->vendor, v->vendor, sizeof(t->vendor) - 1);
            t->vendor[sizeof(t->vendor) - 1] = 0;
            t->vendor_tier = DT_TIER_D;
        }
        dtype_claim(t, v->dtype, DT_TIER_D);
    }

    if (t->band == BAND_BLE) {
        if (t->name[0]) {
            if (sig_match_str(SIG_BLE_CAM, N_SIG_BLE_CAM, t->name, &w) && w == SIG_STRONG) {
                t->evidence |= E_BLE_CAM_NAME;
                dtype_claim(t, DTYPE_CAMERA, DT_TIER_C);
            }
            if (sig_match_str(SIG_BLE_MIC, N_SIG_BLE_MIC, t->name, &w) && w == SIG_STRONG) {
                t->evidence |= E_BLE_MIC_NAME;
                dtype_claim(t, DTYPE_MIC, DT_TIER_C);
            }
        }
        for (uint8_t i = 0; i < t->n_svc16; i++)
            for (uint32_t k = 0; k < N_SIG_SVC16; k++)
                if (t->svc16[i] == SIG_SVC16[k]) t->evidence |= E_BLE_SVC_UUID;
        if (t->have_company)
            for (uint32_t k = 0; k < N_SIG_COMPANY; k++)
                if (t->company_id == SIG_COMPANY[k]) t->evidence |= E_BLE_COMPANY;
    }

    int total = 0;
    for (uint8_t b = 0; b < 32; b++) {
        uint32_t bit = 1u << b;
        if (t->evidence & bit) total += score_for(bit);
    }

    // Proximity as a MULTIPLIER, not an additive term. An out-of-room device
    // with a perfect signature match must not climb the list and crowd out
    // the thing sitting in the smoke detector.
    if (t->tier_best == TIER_AMBIENT)   total /= 4;
    else if (t->tier_best == TIER_NEAR) total = (total * 3) / 4;
    // CONTACT keeps full weight and already carries its own +25.

    if (total > 100) total = 100;
    if (total < 0)   total = 0;
    t->score = (uint8_t)total;
}

static confidence_t classify_confidence(const track_t *t)
{
    if (!t) return CONF_LOW;
    if (t->score >= SCORE_HIGH)     return CONF_HIGH;
    if (t->score >= SCORE_LIKELY)   return CONF_LIKELY;
    if (t->score >= SCORE_POSSIBLE) return CONF_POSSIBLE;
    return CONF_LOW;
}

static const char *confidence_name(confidence_t c)
{
    return c == CONF_HIGH ? "HIGH" : c == CONF_LIKELY ? "LIKELY"
         : c == CONF_POSSIBLE ? "POSSIBLE" : "low";
}

// Human-readable evidence for the detail screen. The operator must be able to
// see WHY, not just a score they have to take on faith.
static uint8_t classify_explain(const track_t *t, char lines[][44], uint8_t maxl)
{
    uint8_t n = 0;
    if (!t) return 0;
    #define ADD(...) do { if (n < maxl) snprintf(lines[n++], 44, __VA_ARGS__); } while (0)

    // Lead with the triage verdict and the rule that produced it, so the
    // label can be judged rather than taken on faith.
    ADD("%s: %.34s", risk_name(t->risk), t->risk_why ? t->risk_why : "");
    if (t->dtype != DTYPE_UNKNOWN)
        ADD("type %s (%s)%s%.12s", dtype_name(t->dtype), dtier_name(t->dtype_tier),
            t->dtype_detail ? " " : "", t->dtype_detail ? t->dtype_detail : "");
    if (t->name[0])
        ADD("name %.36s", t->name);
    if (t->vendor[0])
        ADD("vendor %.20s%s", t->vendor, t->vendor_tier == DT_TIER_A ? " (WPS)" : "");
    if (t->model[0])
        ADD("model %.30s", t->model);
    if (t->dist_m >= 0.0f)
        ADD("%s ~%.2fm peak %ddBm", tier_name(t->tier), t->dist_m, (int)t->peak_session);
    else
        ADD("%s peak %ddBm (no TX ref)", tier_name(t->tier), (int)t->peak_session);

    if (t->evidence & E_WPS_DEVTYPE)    ADD("+ declared its type via WPS");
    if (t->evidence & E_BLE_APPEARANCE) ADD("+ declared BLE appearance 0x%04X", t->appearance);
    if (t->evidence & E_CONTACT_TIER)   ADD("+ hit CONTACT: not through a wall");
    if (t->evidence & E_ROOM_ONLY)      ADD("+ in-room only vs corridor");
    if (t->evidence & E_STREAM_PROFILE) ADD("+ streaming %lup/s %lukB up",
                                            (unsigned long)t->pkt_rate,
                                            (unsigned long)(t->bytes_up/1024));
    if (t->evidence & E_SOFTAP_CAM_SSID)ADD("+ camera SoftAP: %.16s", t->name);
    if (t->evidence & E_CAM_OUI_STRONG)
        ADD("+ camera vendor OUI: %.14s", t->vendor[0] ? t->vendor : "?");
    if (t->evidence & E_BLE_CAM_NAME)   ADD("+ BLE camera name: %.16s", t->name);
    if (t->evidence & E_BLE_MIC_NAME)   ADD("+ BLE mic name: %.18s", t->name);
    if (t->evidence & E_BLE_SVC_UUID)   ADD("+ BLE service UUID of interest");
    if (t->evidence & E_BROAD_PEAK)     ADD("+ broad peak (in-room shape)");
    if (t->evidence & E_SHARP_PEAK)     ADD("~ sharp peak: wall or fixture");
    if (t->evidence & E_CAM_OUI_WEAK)   ADD("~ weak OUI/keyword only");
    if (t->evidence & E_WILDCARD_PROBE) ADD("~ broadcast probe, empty SSID");
    if (t->evidence & E_HIDDEN_SSID)    ADD("~ beaconing, hidden SSID");
    if (n == 1) ADD("no identity evidence yet");
    #undef ADD
    return n;
}

// ===========================================================================
// WIFI CAPTURE  -  passive dual-band 802.11
//
// Receive only. No probe requests, no association, no deauth. That keeps the
// tool invisible to whoever placed the device and keeps it uncomplicated in
// the jurisdictions where transmitting is the line that matters.
// ===========================================================================

// (type definitions hoisted to the TYPE DEFINITIONS block near the top -
//  see the note there before moving them back)

#define CAP_QUEUE_LEN 96
#define BLE_QUEUE_LEN 64
// ---------------------------------------------------------------------------
// Management-frame payload queue.
//
// The main capture queue carries only the first 36 bytes of each frame, which
// is all that RSSI, addressing and traffic accounting need, and keeps the
// queue small enough to absorb a data-frame burst.
//
// But everything interesting about a device's IDENTITY lives past byte 36:
// tagged parameters start at exactly offset 36, so the SSID element begins
// there and the WPS information element is further in still. The first build
// tried to read the SSID out of the 36-byte buffer and therefore never
// returned one - not truncated, never parsed at all, for any frame length.
// The entire SoftAP-camera SSID signature path was dead as a result.
//
// So beacons and probe responses get a second, larger copy on their own
// queue. It is short and drops when full on purpose: beacons repeat every
// ~100 ms, so every access point is captured within a channel dwell or two,
// and a dropped copy costs nothing. Data frames never enter this queue.
// ---------------------------------------------------------------------------
#define IE_QUEUE_LEN   12
#define IE_BUF_LEN    320      // reaches the WPS IE in a typical beacon

typedef struct {
    uint8_t  mac[6];
    uint8_t  band;
    uint16_t len;
    uint8_t  buf[IE_BUF_LEN];
} ie_item_t;

static QueueHandle_t s_iq = NULL;
static volatile uint32_t s_ie_seen = 0, s_ie_drop = 0;

// ---------------------------------------------------------------------------
// Tagged-parameter walk
// ---------------------------------------------------------------------------
#define IE_SSID        0x00
#define IE_VENDOR      0xDD

// WPS lives in a vendor-specific IE with the Wi-Fi Alliance OUI and type 4.
static const uint8_t WPS_OUI_TYPE[4] = { 0x00, 0x50, 0xF2, 0x04 };

#define WPS_ATTR_DEV_NAME      0x1011
#define WPS_ATTR_MANUFACTURER  0x1021
#define WPS_ATTR_MODEL_NAME    0x1023
#define WPS_ATTR_PRIMARY_DEV   0x1054

static void copy_printable(char *dst, size_t cap, const uint8_t *src, uint16_t n)
{
    if (n > cap - 1) n = (uint16_t)(cap - 1);
    uint16_t k = 0;
    for (uint16_t i = 0; i < n; i++) {
        uint8_t c = src[i];
        if (c >= 0x20 && c <= 0x7E) dst[k++] = (char)c;
    }
    dst[k] = 0;
    // Trim trailing spaces - vendors pad these fields.
    while (k && dst[k-1] == ' ') dst[--k] = 0;
}

// Parse one WPS information element body (after the 4-byte OUI+type).
static void parse_wps(track_t *t, const uint8_t *d, uint16_t dlen)
{
    uint16_t i = 0;
    while (i + 4 <= dlen) {
        uint16_t attr = (uint16_t)((d[i] << 8) | d[i+1]);
        uint16_t alen = (uint16_t)((d[i+2] << 8) | d[i+3]);
        if (i + 4 + alen > dlen) break;          // truncated, stop
        const uint8_t *v = &d[i+4];

        switch (attr) {
            case WPS_ATTR_MANUFACTURER:
                // An exact vendor string straight out of the beacon. Better
                // than any OUI table, and it costs one memcpy.
                if (alen && !t->vendor[0]) {
                    copy_printable(t->vendor, sizeof(t->vendor), v, alen);
                    if (t->vendor[0]) t->vendor_tier = DT_TIER_A;
                }
                break;

            case WPS_ATTR_MODEL_NAME:
                if (alen && !t->model[0])
                    copy_printable(t->model, sizeof(t->model), v, alen);
                break;

            case WPS_ATTR_DEV_NAME:
                // Only used as a name if nothing better arrived; the SSID is
                // usually more recognisable to the operator.
                if (alen && !t->name[0])
                    copy_printable(t->name, sizeof(t->name), v, alen);
                break;

            case WPS_ATTR_PRIMARY_DEV:
                // 2-byte category, 4-byte OUI, 2-byte subcategory. Only the
                // predefined Wi-Fi Alliance OUI is interpreted; a
                // vendor-specific OUI makes the subcategory meaningless.
                if (alen >= 8 && memcmp(v + 2, WPS_OUI_TYPE, 4) == 0) {
                    uint16_t cat = (uint16_t)((v[0] << 8) | v[1]);
                    uint16_t sub = (uint16_t)((v[6] << 8) | v[7]);
                    const char *detail = NULL;
                    dtype_t d2 = wps_category_to_dtype(cat, sub, &detail);
                    if (d2 != DTYPE_UNKNOWN) {
                        dtype_claim(t, d2, DT_TIER_A);
                        t->evidence |= E_WPS_DEVTYPE;
                        if (detail) t->dtype_detail = detail;
                    }
                }
                break;

            default:
                break;
        }
        i = (uint16_t)(i + 4 + alen);
    }
}

// Walk the tagged parameters of a beacon or probe response.
static void parse_mgmt_ies(track_t *t, const uint8_t *f, uint16_t len)
{
    // 24-byte MAC header + 12-byte fixed beacon body, then tagged params.
    uint16_t i = 36;
    while (i + 2 <= len) {
        uint8_t id = f[i], elen = f[i+1];
        if (i + 2 + elen > len) break;
        const uint8_t *d = &f[i+2];

        if (id == IE_SSID) {
            if (elen == 0) {
                t->evidence |= E_HIDDEN_SSID;
            } else {
                char ssid[NAME_LEN];
                copy_printable(ssid, sizeof(ssid), d, elen);
                if (ssid[0]) { strncpy(t->name, ssid, NAME_LEN - 1); t->name[NAME_LEN-1] = 0; }
            }
        } else if (id == IE_VENDOR && elen >= 4 &&
                   memcmp(d, WPS_OUI_TYPE, 4) == 0) {
            parse_wps(t, d + 4, (uint16_t)(elen - 4));
        }
        i = (uint16_t)(i + 2 + elen);
    }
}

static void ie_pump(uint32_t now)
{
    (void)now;
    if (!s_iq) return;
    static ie_item_t it;                 // 330 bytes: static, not on the stack
    for (int n = 0; n < 4; n++) {        // a few per tick; parsing is not urgent
        if (xQueueReceive(s_iq, &it, 0) != pdTRUE) break;
        track_t *t = track_get(it.mac, (rband_t)it.band, false);
        if (!t) continue;                // gone already, nothing to attach to
        parse_mgmt_ies(t, it.buf, it.len);
    }
}


static QueueHandle_t s_wq = NULL;
static volatile uint32_t s_frames = 0, s_frames_drop = 0;
static rband_t s_band = BAND_24;
static uint8_t s_channel = 1;
static volatile bool s_wifi_run = false;
static uint8_t s_ch5[sizeof(CH_5_CANDIDATES)];
static uint8_t s_n_ch5 = 0;
static bool    s_dfs_ok = false;

#define FC_TYPE(fc)     (((fc) >> 2) & 0x03)
#define FC_SUBTYPE(fc)  (((fc) >> 4) & 0x0F)
#define FC_TO_DS(f2)    ((f2) & 0x01)
#define FC_FROM_DS(f2)  (((f2) >> 1) & 0x01)
#define TYPE_MGMT 0
#define TYPE_DATA 2
#define ST_PROBE_REQ  0x04
#define ST_PROBE_RESP 0x05
#define ST_BEACON     0x08

static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// The promiscuous callback runs inside the WiFi driver task, so it must post
// and return. Anything heavier stalls the radio and costs us frames.
static void promisc_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (!s_wifi_run || !s_wq) return;
    if (type == WIFI_PKT_MISC) return;

    const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
    s_frames++;

    cap_item_t it;
    uint16_t copy = p->rx_ctrl.sig_len;
    if (copy > sizeof(it.hdr)) copy = sizeof(it.hdr);
    memcpy(it.hdr, p->payload, copy);
    if (copy < sizeof(it.hdr)) memset(it.hdr + copy, 0, sizeof(it.hdr) - copy);
    it.len     = p->rx_ctrl.sig_len;
    it.rssi    = (int8_t)p->rx_ctrl.rssi;
    it.channel = s_channel;
    it.band    = (uint8_t)s_band;

    if (xQueueSend(s_wq, &it, 0) != pdTRUE) s_frames_drop++;

    // Beacons and probe responses carry the device's own identity past byte
    // 36 - SSID, and often WPS manufacturer, model and declared device type.
    // Copy those to the IE queue. Dropping when full is fine: beacons repeat
    // every ~100 ms, so nothing is permanently lost.
    uint8_t type_ = FC_TYPE(it.hdr[0]), sub_ = FC_SUBTYPE(it.hdr[0]);
    if (type_ == TYPE_MGMT && (sub_ == ST_BEACON || sub_ == ST_PROBE_RESP) &&
        s_iq && p->rx_ctrl.sig_len > 36) {
        static ie_item_t ie;            // driver task, so not on the stack
        memcpy(ie.mac, p->payload + 10, 6);           // addr2, the transmitter
        ie.band = (uint8_t)s_band;
        uint16_t n = p->rx_ctrl.sig_len;
        if (n > IE_BUF_LEN) n = IE_BUF_LEN;
        memcpy(ie.buf, p->payload, n);
        ie.len = n;
        s_ie_seen++;
        if (xQueueSend(s_iq, &ie, 0) != pdTRUE) s_ie_drop++;
    }
}

static void wifi_cap_init()
{
    if (!s_wq) s_wq = xQueueCreate(CAP_QUEUE_LEN, sizeof(cap_item_t));
    if (!s_iq) s_iq = xQueueCreate(IE_QUEUE_LEN, sizeof(ie_item_t));
    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect(true, false);
    esp_wifi_set_ps(WIFI_PS_NONE);      // power save would gate our RX

    wifi_promiscuous_filter_t f = {};
    // Management for beacons/probes, data for the streaming heuristic.
    // Control frames are high-volume and carry no usable addr2 identity, so
    // filtering them keeps the queue for frames we can act on.
    f.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
    esp_wifi_set_promiscuous_filter(&f);
    esp_wifi_set_promiscuous_rx_cb(&promisc_cb);
}

static bool wifi_cap_set_band(rband_t band)
{
    if (band == s_band) return true;
    if (band != BAND_24 && band != BAND_5) return false;
    // NOTE: the band-mode API arrived with C5 support. If your core exposes a
    // different symbol, THIS is the one call to adjust.
    wifi_band_mode_t m = (band == BAND_5) ? WIFI_BAND_MODE_5G_ONLY
                                          : WIFI_BAND_MODE_2G_ONLY;
    if (esp_wifi_set_band_mode(m) != ESP_OK) return false;
    s_band = band;
    vTaskDelay(pdMS_TO_TICKS(BAND_SWITCH_SETTLE_MS));
    return true;
}

static bool wifi_cap_set_channel(uint8_t ch)
{
    if (esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
    s_channel = ch;
    return true;
}

// Probe which 5 GHz channels the driver will actually accept, rather than
// assuming. The C5 supports DFS channels but only PASSIVE radar detection, so
// the regulatory table may refuse 52-144 depending on country config. We
// measure once at boot and report it on the SETUP screen.
static void wifi_cap_probe_5g()
{
    s_n_ch5 = 0; s_dfs_ok = false;
    bool was = s_wifi_run;
    s_wifi_run = false;                 // ignore frames while we thrash

    if (!wifi_cap_set_band(BAND_5)) { s_wifi_run = was; return; }

    for (uint32_t i = 0; i < sizeof(CH_5_CANDIDATES); i++) {
        uint8_t ch = CH_5_CANDIDATES[i];
        if (esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) == ESP_OK) {
            s_ch5[s_n_ch5++] = ch;
            if (ch >= 52 && ch <= 144) s_dfs_ok = true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    wifi_cap_set_band(BAND_24);
    wifi_cap_set_channel(CH_24[0]);
    s_wifi_run = was;
}

static void wifi_cap_start() { esp_wifi_set_promiscuous(true);  s_wifi_run = true; }
static void wifi_cap_stop()
{
    s_wifi_run = false;
    esp_wifi_set_promiscuous(false);
    // Drain, or a stale burst lands in the next pass tagged with the wrong
    // band and channel.
    if (s_wq) xQueueReset(s_wq);
    if (s_iq) xQueueReset(s_iq);
}

// SSID and WPS parsing now happen in parse_mgmt_ies() against the full
// payload on the IE queue. The old parse_ssid() that lived here read from the
// 36-byte header copy, where the tagged parameters have not started yet, and
// so returned nothing for every frame at every length.

static void handle_frame(const cap_item_t *it, uint32_t now)
{
    const uint8_t *f = it->hdr;
    uint8_t fc1 = f[0], fc2 = f[1];
    uint8_t type = FC_TYPE(fc1), sub = FC_SUBTYPE(fc1);
    const uint8_t *addr1 = f + 4, *addr2 = f + 10;
    rband_t band = (rband_t)it->band;

    // RSSI belongs to the radio that TRANSMITTED, which is addr2 and only
    // addr2. See the attribution rule on track_observe_rssi.
    track_t *tx = NULL;
    if (memcmp(addr2, BCAST, 6) != 0) {
        tx = track_get(addr2, band, true);
        if (tx) { tx->channel = it->channel; track_observe_rssi(tx, it->rssi, now); }
    }

    if (type == TYPE_MGMT) {
        if (!tx) return;
        if (sub == ST_BEACON || sub == ST_PROBE_RESP) {
            tx->is_ap = true;
            if (sub == ST_BEACON) tx->n_beacon++; else tx->n_probe_resp++;
            // Identity is filled in by ie_pump() from the full payload.
        } else if (sub == ST_PROBE_REQ) {
            tx->n_probe_req++;
            // A probe request's SSID element sits at offset 24, inside the
            // header copy we do have. Zero length means a broadcast probe:
            // a device hunting for any network it knows, common on
            // unprovisioned cameras.
            if (it->len > 25 && f[24] == IE_SSID && f[25] == 0)
                tx->evidence |= E_WILDCARD_PROBE;
        } else tx->n_mgmt_other++;
        return;
    }

    if (type == TYPE_DATA) {
        bool to_ds = FC_TO_DS(fc2), from_ds = FC_FROM_DS(fc2);
        if (to_ds && !from_ds) {
            // Station -> AP. addr2 is the station: this is the uplink that
            // drives the streaming heuristic.
            if (tx) { tx->n_data_up++; tx->bytes_up += it->len; }
        } else if (!to_ds && from_ds) {
            // AP -> station. addr2 is the AP (correctly gets the RSSI) and
            // addr1 is the station. Credit the station's downlink bytes
            // WITHOUT touching its RSSI - this is how we notice a camera that
            // is currently only receiving. Traffic shape, never proximity.
            if (tx) tx->is_ap = true;
            if (memcmp(addr1, BCAST, 6) != 0) {
                track_t *rx = track_get(addr1, band, true);
                if (rx) {
                    rx->n_data_down++;
                    rx->bytes_down += it->len;
                    rx->last_ms = now;
                    if (rx->first_ms == 0) rx->first_ms = now;
                }
            }
        } else if (!to_ds && !from_ds) {
            if (tx) { tx->n_data_up++; tx->bytes_up += it->len; }
        }
    }
}

static void wifi_cap_pump(uint32_t now)
{
    if (!s_wq) return;
    cap_item_t it;
    for (int i = 0; i < 64; i++) {       // bounded so the UI still gets time
        if (xQueueReceive(s_wq, &it, 0) != pdTRUE) break;
        handle_frame(&it, now);
    }
}

// ===========================================================================
// BLE CAPTURE  -  passive only
//
// Passive, not active. Active scanning would fetch scan responses (more name
// and UUID data) but it transmits a scan request, making the tool visible.
// Not worth it.
//
// The important job is extracting a calibrated transmit-power reference where
// one exists, because consumer BLE TX power spans ~28 dB and that uncertainty
// otherwise swamps the wall margin the gate depends on.
// ===========================================================================

// (type definitions hoisted to the TYPE DEFINITIONS block near the top -
//  see the note there before moving them back)

static QueueHandle_t s_bq = NULL;
static volatile uint32_t s_adv = 0, s_adv_drop = 0;
static uint32_t s_with_ref = 0;
static volatile bool s_ble_run = false;
static NimBLEScan *s_scan = NULL;

#define AD_APPEARANCE      0x19
#define AD_UUID16_SOME     0x02
#define AD_UUID16_ALL      0x03
#define AD_NAME_SHORT      0x08
#define AD_NAME_COMPLETE   0x09
#define AD_TX_POWER        0x0A
#define AD_SERVICE_DATA_16 0x16
#define AD_MFR_DATA        0xFF

class SweepScanCB : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override {
        if (!s_ble_run || !s_bq) return;
        s_adv++;
        ble_item_t it;
        memset(&it, 0, sizeof(it));

        const uint8_t *a = dev->getAddress().getBase()->val;
        // NimBLE stores addresses little-endian; present big-endian so it
        // matches every other tool.
        for (int i = 0; i < 6; i++) it.mac[i] = a[5 - i];
        it.addr_type = dev->getAddress().getType();
        it.rssi = (int8_t)dev->getRSSI();

        // Copy the raw payload and walk it ourselves. NimBLE's accessors are
        // convenient but hide TX power and the beacon ranging fields, which
        // are exactly what we need most.
        std::vector<uint8_t> pl = dev->getPayload();
        size_t n = pl.size();
        if (n > sizeof(it.adv)) n = sizeof(it.adv);
        if (n) memcpy(it.adv, pl.data(), n);
        it.adv_len = (uint8_t)n;

        if (xQueueSend(s_bq, &it, 0) != pdTRUE) s_adv_drop++;
    }
};
static SweepScanCB s_cb;

static void ble_cap_init()
{
    if (!s_bq) s_bq = xQueueCreate(BLE_QUEUE_LEN, sizeof(ble_item_t));
    NimBLEDevice::init("");
    NimBLEDevice::setPower(ESP_PWR_LVL_N12);   // we never advertise; stay quiet
    s_scan = NimBLEDevice::getScan();
    s_scan->setScanCallbacks(&s_cb, false);
    s_scan->setActiveScan(false);              // PASSIVE - never transmit
    // Window == interval means continuous listening, which is the whole point
    // of a BLE-exclusive pass: a 10.24 s advertiser needs every millisecond.
    s_scan->setInterval(160);                  // 160 * 0.625 ms = 100 ms
    s_scan->setWindow(160);
    s_scan->setDuplicateFilter(false);         // we need every advert for RSSI
    s_scan->setMaxResults(0);                  // stream via callback
}

static void ble_cap_start()
{
    if (!s_scan) return;
    s_ble_run = true;
    s_scan->start(0, false, true);             // 0 = continuous
}
static void ble_cap_stop()
{
    s_ble_run = false;
    if (s_scan) s_scan->stop();
    if (s_bq) xQueueReset(s_bq);
}

static void handle_advert(const ble_item_t *it, uint32_t now)
{
    track_t *t = track_get(it->mac, BAND_BLE, true);
    if (!t) return;
    t->addr_type = it->addr_type;
    track_observe_rssi(t, it->rssi, now);

    bool had_ref = (t->ref.src != REF_NONE);
    size_t i = 0;
    while (i + 1 < it->adv_len) {
        uint8_t len = it->adv[i];
        if (len == 0) break;
        if (i + 1 + len > it->adv_len) break;       // truncated, stop
        uint8_t type = it->adv[i+1];
        const uint8_t *d = &it->adv[i+2];
        uint8_t dlen = (uint8_t)(len - 1);

        if (type == AD_NAME_COMPLETE || type == AD_NAME_SHORT) {
            // Never let a short name overwrite a complete one.
            if (!(t->name[0] && type == AD_NAME_SHORT)) {
                uint8_t cp = dlen < (NAME_LEN-1) ? dlen : (NAME_LEN-1);
                memcpy(t->name, d, cp); t->name[cp] = 0;
                for (uint8_t k = 0; k < cp; k++)
                    if (t->name[k] < 0x20 || t->name[k] > 0x7E) t->name[k] = '.';
            }
        } else if (type == AD_APPEARANCE) {
            // The device declaring its own category. Audio Source means a
            // microphone; Audio Sink means a speaker. Conflating the two
            // would put every Bluetooth speaker at the top of the list.
            if (dlen >= 2) {
                uint16_t ap = (uint16_t)(d[0] | (d[1] << 8));
                t->appearance = ap;
                dtype_t dt = appearance_to_dtype(ap);
                if (dt != DTYPE_UNKNOWN) {
                    dtype_claim(t, dt, DT_TIER_A);
                    t->evidence |= E_BLE_APPEARANCE;
                }
            }
        } else if (type == AD_TX_POWER) {
            if (dlen >= 1 && t->ref.src == REF_NONE)
                t->ref = prox_ref_from_txpower((int8_t)d[0]);
        } else if (type == AD_UUID16_SOME || type == AD_UUID16_ALL) {
            for (uint8_t k = 0; k + 1 < dlen && t->n_svc16 < 4; k += 2) {
                uint16_t u = (uint16_t)(d[k] | (d[k+1] << 8));
                bool dup = false;
                for (uint8_t m = 0; m < t->n_svc16; m++) if (t->svc16[m] == u) dup = true;
                if (!dup) t->svc16[t->n_svc16++] = u;
                // A SIG microphone or audio-input service means the device
                // has a microphone in it, declared by the device itself.
                dtype_t du = svc_uuid_to_dtype(u);
                if (du != DTYPE_UNKNOWN) dtype_claim(t, du, DT_TIER_A);
            }
        } else if (type == AD_SERVICE_DATA_16 && dlen >= 4) {
            uint16_t svc = (uint16_t)(d[0] | (d[1] << 8));
            if (svc == 0xFEAA && (d[2] == 0x00 || d[2] == 0x10)) {
                // Eddystone UID/URL: better reference than a bare TX power
                // field, so allow it to upgrade.
                if (t->ref.src == REF_NONE || t->ref.src == REF_ADV_TXPOWER)
                    t->ref = prox_ref_from_eddystone((int8_t)d[3]);
            }
        } else if (type == AD_MFR_DATA && dlen >= 2) {
            t->company_id = (uint16_t)(d[0] | (d[1] << 8));
            t->have_company = true;
            // iBeacon: company 0x004C, type 0x02, len 0x15, then 16-byte UUID
            // + major + minor + measured power. That measured power IS the
            // calibrated RSSI at 1 m, so it beats everything else.
            if (t->company_id == 0x004C && dlen >= 25 && d[2] == 0x02 && d[3] == 0x15)
                t->ref = prox_ref_from_ibeacon((int8_t)d[24]);
        }
        i += (size_t)len + 1;
    }
    if (!had_ref && t->ref.src != REF_NONE) s_with_ref++;
}

static void ble_cap_pump(uint32_t now)
{
    if (!s_bq) return;
    ble_item_t it;
    for (int i = 0; i < 48; i++) {
        if (xQueueReceive(s_bq, &it, 0) != pdTRUE) break;
        handle_advert(&it, now);
    }
}

// ===========================================================================
// SCHEDULER  -  separate passes, plus dwell accounting
//
// Three hard constraints on this chip:
//   1. no simultaneous dual-band - esp_wifi_set_band_mode picks one
//   2. one antenna on WROOM-1, software time-division, no GPIO switching
//   3. Wi-Fi and BLE also share time by coexistence
//
// So detection probability at a sweep position is radio-on dwell versus the
// target's transmit interval. BLE adverts range 20 ms to 10.24 s. A three-way
// round-robin at 33% duty needs 30+ s of wall clock per position to cover a
// 10 s advertiser - unusable while walking.
//
// MODE_SWEEP_ALL is therefore NOT a simultaneous sweep, because the hardware
// cannot do one. It is two sequential passes with an explicit prompt between
// them: Wi-Fi across the whole room, then BLE across the whole room again.
// Two fast walks beat one slow one. Round-robin survives only in WATCH, where
// the board sits still and dwell is unbounded.
// ===========================================================================

static sweep_mode_t s_mode = MODE_IDLE;
static rband_t  s_active = BAND_24;
static uint8_t  s_ch24_idx = 0, s_ch5_idx = 0, s_watch_phase = 0;
static uint32_t s_last_hop = 0, s_band_slice = 0, s_watch_slice = 0;
static uint32_t s_pos_start = 0, s_radio_ms[BAND_COUNT], s_last_accrue = 0;
static int8_t   s_last_strongest = INT8_MIN;
static uint32_t s_positions = 0;

// SWEEP ALL state: phase 0 = Wi-Fi pass, phase 1 = BLE pass.
static uint8_t  s_all_phase = 0;
static bool     s_pass_prompt = false;
static uint32_t s_pass_radio_ms = 0;
// Auto-advance from the Wi-Fi pass to the BLE pass after this much radio time,
// so an operator who never taps the button still gets both passes. Generous:
// a room takes a couple of minutes to walk properly.
#define ALL_PASS_AUTO_MS 150000

static const char *sched_mode_name(sweep_mode_t m)
{
    return m == MODE_WIFI_SWEEP ? "WI-FI"
         : m == MODE_BLE_SWEEP  ? "BLE"
         : m == MODE_SWEEP_ALL  ? "ALL"
         : m == MODE_WATCH      ? "WATCH" : "IDLE";
}

// Which protocol currently owns the radio.
static rband_t sched_active_proto()
{
    if (s_mode == MODE_BLE_SWEEP) return BAND_BLE;
    if (s_mode == MODE_SWEEP_ALL && s_all_phase == 1) return BAND_BLE;
    if (s_mode == MODE_WATCH && s_watch_phase == 2) return BAND_BLE;
    return s_active;
}

// Compact status for the sweep banner, e.g. "ALL 1/2 2.4G" or "BLE".
static const char *sched_status()
{
    static char buf[20];
    switch (s_mode) {
        case MODE_SWEEP_ALL:
            if (s_all_phase == 0)
                snprintf(buf, sizeof(buf), "ALL 1/2 %s", band_name(s_active));
            else
                snprintf(buf, sizeof(buf), "ALL 2/2 BLE");
            break;
        case MODE_WIFI_SWEEP:
            snprintf(buf, sizeof(buf), "WI-FI %s", band_name(s_active));
            break;
        case MODE_BLE_SWEEP:
            snprintf(buf, sizeof(buf), "BLE");
            break;
        case MODE_WATCH:
            snprintf(buf, sizeof(buf), "WATCH %s", band_name(sched_active_proto()));
            break;
        default:
            snprintf(buf, sizeof(buf), "IDLE");
            break;
    }
    return buf;
}

static void sched_mark_position(uint32_t now)
{
    // Snapshot per-position peaks into each track's sweep profile before
    // resetting, so the peak-shape classifier has data to work with.
    if (s_pos_start) { tracks_close_position(now); s_positions++; }
    s_pos_start = now;
    s_last_accrue = now;
    memset(s_radio_ms, 0, sizeof(s_radio_ms));
    s_last_strongest = INT8_MIN;
}

static void radios_wifi_only()
{
    ble_cap_stop();
    wifi_cap_start();
    if (wifi_cap_set_band(BAND_24)) s_active = BAND_24;
    wifi_cap_set_channel(CH_24[0]);
}

static void radios_ble_only()
{
    wifi_cap_stop();
    s_active = BAND_BLE;
    ble_cap_start();
}

static void sched_set_mode(sweep_mode_t m)
{
    if (m == s_mode) return;
    wifi_cap_stop();
    ble_cap_stop();
    s_mode = m;

    uint32_t now = millis();
    s_band_slice = s_watch_slice = s_last_hop = now;
    s_watch_phase = 0;
    s_all_phase = 0;
    s_pass_radio_ms = 0;
    s_pass_prompt = false;
    s_positions = 0;
    s_pos_start = 0;
    sched_mark_position(now);

    if (m == MODE_WIFI_SWEEP || m == MODE_WATCH || m == MODE_SWEEP_ALL)
        radios_wifi_only();
    else if (m == MODE_BLE_SWEEP)
        radios_ble_only();
    else
        led_rgb(0, 0, 20);      // idle: blue
}

// Roll SWEEP ALL from the Wi-Fi pass to the BLE pass. The prompt matters: the
// operator has to physically walk the room a second time, and without being
// told they will assume one lap covered everything.
static void sched_all_next_pass(uint32_t now)
{
    if (s_mode != MODE_SWEEP_ALL || s_all_phase != 0) return;
    s_all_phase = 1;
    s_pass_radio_ms = 0;
    s_pass_prompt = true;
    radios_ble_only();
    sched_mark_position(now);
}

static void sched_accrue(uint32_t now)
{
    if (!s_last_accrue) { s_last_accrue = now; return; }
    uint32_t dt = now - s_last_accrue;
    s_last_accrue = now;
    if (s_mode == MODE_IDLE) return;
    rband_t p = sched_active_proto();
    if (p < BAND_COUNT) s_radio_ms[p] += dt;
    s_pass_radio_ms += dt;
}

static uint32_t sched_position_radio_ms()
{
    switch (s_mode) {
        case MODE_BLE_SWEEP: return s_radio_ms[BAND_BLE];
        case MODE_WIFI_SWEEP: return s_radio_ms[BAND_24] + s_radio_ms[BAND_5];
        case MODE_SWEEP_ALL:
            return s_all_phase == 0 ? s_radio_ms[BAND_24] + s_radio_ms[BAND_5]
                                    : s_radio_ms[BAND_BLE];
        case MODE_WATCH: {
            // The meaningful figure is the worst-served protocol, since that
            // is what actually bounds coverage.
            uint32_t worst = s_radio_ms[0];
            for (int b = 1; b < BAND_COUNT; b++)
                if (s_radio_ms[b] < worst) worst = s_radio_ms[b];
            return worst;
        }
        default: return 0;
    }
}

// The longest transmit interval we can honestly claim to cover here: radio
// time divided by the packets the gate requires.
static uint32_t sched_covered_interval_ms()
{
    uint32_t r = sched_position_radio_ms();
    return r ? r / MIN_PKTS_FOR_NEAR : 0;
}

static bool sched_sweep_too_fast()
{
    if (s_mode == MODE_IDLE || !s_pos_start) return false;
    return sched_position_radio_ms() < SWEEP_WARN_BELOW_MS;
}

// Movement proxy. There is no IMU, so if the strongest thing in earshot
// changed level substantially the operator almost certainly moved. Only
// auto-remark once the position has earned enough radio time to be worth
// recording, or a noisy room would shred the sweep profile.
static void sched_note_strongest(int8_t peak, uint32_t now)
{
    if (peak == INT8_MIN) return;
    if (s_last_strongest == INT8_MIN) { s_last_strongest = peak; return; }
    if (abs((int)peak - (int)s_last_strongest) >= AUTO_REMARK_DELTA_DB &&
        sched_position_radio_ms() >= SWEEP_WARN_BELOW_MS) {
        sched_mark_position(now);
        s_last_strongest = peak;
    } else if (peak > s_last_strongest) s_last_strongest = peak;
}

static void wifi_rotate(uint32_t now)
{
    if ((uint32_t)(now - s_last_hop) >= WIFI_CH_DWELL_MS) {
        s_last_hop = now;
        if (s_active == BAND_24) {
            s_ch24_idx = (uint8_t)((s_ch24_idx + 1) % (sizeof(CH_24)/sizeof(CH_24[0])));
            wifi_cap_set_channel(CH_24[s_ch24_idx]);
        } else if (s_n_ch5) {
            s_ch5_idx = (uint8_t)((s_ch5_idx + 1) % s_n_ch5);
            wifi_cap_set_channel(s_ch5[s_ch5_idx]);
        }
    }
    // Band switching is not free, so swap on a coarse slice, not per channel.
    if ((uint32_t)(now - s_band_slice) >= WIFI_BAND_SLICE_MS) {
        s_band_slice = now;
        if (s_active == BAND_24 && s_n_ch5) {
            if (wifi_cap_set_band(BAND_5)) {
                s_active = BAND_5; s_ch5_idx = 0; wifi_cap_set_channel(s_ch5[0]);
            }
        } else if (wifi_cap_set_band(BAND_24)) {
            s_active = BAND_24; s_ch24_idx = 0; wifi_cap_set_channel(CH_24[0]);
        }
        s_last_hop = now;
    }
}

static void watch_rotate(uint32_t now)
{
    if ((uint32_t)(now - s_watch_slice) < WATCH_SLICE_MS) {
        if (s_watch_phase != 2) wifi_rotate(now);
        return;
    }
    s_watch_slice = now;
    s_watch_phase = (uint8_t)((s_watch_phase + 1) % 3);
    if (s_watch_phase == 0) {
        radios_wifi_only();
    } else if (s_watch_phase == 1) {
        ble_cap_stop();
        wifi_cap_start();
        if (s_n_ch5 && wifi_cap_set_band(BAND_5)) {
            s_active = BAND_5; wifi_cap_set_channel(s_ch5[0]);
        }
    } else {
        radios_ble_only();
    }
    s_last_hop = now;
}

static void sched_tick(uint32_t now)
{
    sched_accrue(now);

    switch (s_mode) {
        case MODE_WIFI_SWEEP:
            wifi_rotate(now);
            break;
        case MODE_SWEEP_ALL:
            if (s_all_phase == 0) {
                wifi_rotate(now);
                if (s_pass_radio_ms >= ALL_PASS_AUTO_MS) sched_all_next_pass(now);
            }
            break;
        case MODE_WATCH:
            watch_rotate(now);
            break;
        default:
            break;      // BLE_SWEEP has nothing to rotate: BLE owns the radio
    }
}

// ===========================================================================
// CALIBRATION  -  guided, two distances, per band
//
// The shipped thresholds are maths plus an assumed margin. Antenna choice,
// whether the 0-ohm RF selector actually routes to your U.FL socket, and
// board-to-board variation all move them several dB.
// ===========================================================================

// (type definitions hoisted to the TYPE DEFINITIONS block near the top -
//  see the note there before moving them back)

#define CAL_MIN_SAMPLES 40
#define CAL_MAX_SAMPLES 200

static cal_state_t s_cal = CAL_IDLE;
static rband_t  s_cal_band = BAND_24;
static int8_t   s_cal_buf[CAL_MAX_SAMPLES];
static uint16_t s_cal_n = 0;
static int8_t   s_cal_near = 0, s_cal_contact = 0;

static int cmp_i8(const void *a, const void *b)
{ return (int)(*(const int8_t*)a) - (int)(*(const int8_t*)b); }

// The gate fires on peak, so the threshold should sit where the peak of a
// genuine target at the reference distance lands - not at its mean. The 80th
// percentile is high enough to represent peak behaviour, low enough that one
// multipath spike does not set the threshold.
static int8_t cal_p80()
{
    if (!s_cal_n) return 0;
    int8_t tmp[CAL_MAX_SAMPLES];
    memcpy(tmp, s_cal_buf, s_cal_n);
    qsort(tmp, s_cal_n, 1, cmp_i8);
    uint16_t i = (uint16_t)((s_cal_n * 80) / 100);
    if (i >= s_cal_n) i = (uint16_t)(s_cal_n - 1);
    return tmp[i];
}

static void cal_feed(int8_t rssi)
{
    if (s_cal != CAL_NEAR && s_cal != CAL_CONTACT) return;
    if (rssi == INT8_MIN) return;
    if (s_cal_n < CAL_MAX_SAMPLES) s_cal_buf[s_cal_n++] = rssi;
}

static void cal_advance()
{
    if (s_cal == CAL_IDLE)      { s_cal = CAL_NEAR; s_cal_n = 0; s_cal_band = s_active; }
    else if (s_cal == CAL_NEAR) {
        if (s_cal_n < CAL_MIN_SAMPLES) { s_cal = CAL_FAILED; return; }
        s_cal_near = cal_p80(); s_cal_n = 0; s_cal = CAL_CONTACT;
    } else if (s_cal == CAL_CONTACT) {
        if (s_cal_n < CAL_MIN_SAMPLES) { s_cal = CAL_FAILED; return; }
        s_cal_contact = cal_p80();
        // Free space between 2.0 m and 0.3 m is 16.5 dB. Much less than 8 dB
        // means the reference never moved, or the antenna is not connected.
        if (s_cal_contact - s_cal_near < 8) { s_cal = CAL_FAILED; return; }
        s_cal = CAL_DONE;
    } else if (s_cal == CAL_DONE) {
        g_cal.near_thresh[s_cal_band]    = s_cal_near;
        g_cal.contact_thresh[s_cal_band] = s_cal_contact;
        g_cal.calibrated[s_cal_band]     = true;
        prox_cal_save();
        s_cal = CAL_IDLE;
    } else s_cal = CAL_IDLE;             // clear a failure
}

static const char *cal_prompt()
{
    switch (s_cal) {
        case CAL_NEAR:    return "Ref at 2.0m. Hold still.";
        case CAL_CONTACT: return "Move ref to 0.3m. Hold.";
        case CAL_DONE:    return "Done - tap to commit.";
        case CAL_FAILED:  return "FAILED: few samples/bad spread";
        default:          return "Tap to calibrate this band";
    }
}

// ===========================================================================
// SD LOGGING
//
// MACs are written as a session-salted hash. The salt is random per boot and
// never persisted, so the same device gets a different pseudonym next
// session: enough to correlate within one sweep, without building a permanent
// record of every device belonging to everyone in the building.
//
// Pseudonymisation is not anonymity - names, SSIDs and manufacturer payloads
// can still be distinctive. Treat exported logs as sensitive.
//
// Frame metadata only. Never payloads.
// ===========================================================================

static bool  s_sd_ok = false;
static char  s_fname[40];
static File  s_file;
static uint8_t s_salt[16];

// Strip anything that would break a CSV row. Applied to every free-text
// field, since vendor and model strings come straight off the air.
static void csv_safe(char *dst, size_t cap, const char *src)
{
    size_t k = 0;
    for (size_t i = 0; src[i] && k < cap - 1; i++) {
        char c = src[i];
        if (c == ',' || c == '"' || c == '\n' || c == '\r') c = '_';
        dst[k++] = c;
    }
    dst[k] = 0;
}

static void log_pseudonym(const uint8_t mac[6], char *out, size_t cap)
{
    uint8_t buf[22], h[32];
    memcpy(buf, s_salt, 16);
    memcpy(buf + 16, mac, 6);
    mbedtls_sha256(buf, sizeof(buf), h, 0);
    snprintf(out, cap, "%02x%02x%02x%02x%02x%02x%02x%02x",
             h[0],h[1],h[2],h[3],h[4],h[5],h[6],h[7]);
}

static bool log_init()
{
    esp_fill_random(s_salt, sizeof(s_salt));
    spi_take();
    if (!SD.begin(PIN_SD_CS, SPI, SPI_HZ_SD)) { spi_give(); s_sd_ok = false; return false; }
    if (!SD.exists(LOG_DIR)) SD.mkdir(LOG_DIR);
    // No RTC on this board, so number the sessions rather than timestamp them.
    for (int n = 1; n < 1000; n++) {
        snprintf(s_fname, sizeof(s_fname), LOG_DIR "/vulp_%03d.csv", n);
        if (!SD.exists(s_fname)) break;
    }
    s_file = SD.open(s_fname, FILE_WRITE);
    if (!s_file) { spi_give(); s_sd_ok = false; return false; }
    s_file.println("# Vulpecula RF sweep log. Session-salted MAC hashes;");
    s_file.println("# the salt is random per boot and never persisted.");
    s_file.println("# Frame metadata only - no payloads.");
    s_file.println("event,ms,pseudonym,band,ch,name,tier,peak,median,dist_m,"
                   "score,conf,risk,why,dtype,dtier,vendor,model,"
                   "pkts,rate,up_kB,down_kB,evidence,phase");
    s_file.flush();
    spi_give();
    s_sd_ok = true;
    return true;
}

// Runs on the PUMP task while the UI task is drawing, so it must hold the SPI
// mutex for the whole write - the SD card shares the bus with the display.
static void log_snapshot(uint32_t now, const char *event)
{
    if (!s_sd_ok) return;
    spi_take();
    static track_t *list[MAX_TRACKS];
    uint16_t n = tracks_sorted(list, MAX_TRACKS);
    bool baseline = (strcmp(event, "BASELINE") == 0);

    for (uint16_t i = 0; i < n; i++) {
        track_t *t = list[i];
        // Keep the main log readable: only tracks that crossed the gate or
        // scored. Ambient-only entries stay in the table for the diff and get
        // written on the BASELINE event.
        if (!(t->tier_best >= TIER_NEAR || t->score >= SCORE_POSSIBLE || baseline))
            continue;

        char pseudo[20];
        log_pseudonym(t->mac, pseudo, sizeof(pseudo));
        int8_t med = ring_median(&t->ring, now, MEDIAN_WINDOW_MS);

        char safe[NAME_LEN];
        csv_safe(safe, sizeof(safe), t->name);

        char vsafe[24], msafe[20];
        csv_safe(vsafe, sizeof(vsafe), t->vendor);
        csv_safe(msafe, sizeof(msafe), t->model);

        s_file.printf("%s,%lu,%s,%s,%u,%s,%s,%d,%d,%.2f,%u,%s,"
                      "%s,%s,%s,%s,%s,%s,%lu,%lu,%lu,%lu,0x%08lx,%d\n",
            event, (unsigned long)now, pseudo, band_name(t->band),
            (unsigned)t->channel, safe, tier_name(t->tier_best),
            (int)t->peak_session, (int)(med == INT8_MIN ? 0 : med),
            (double)t->dist_m, (unsigned)t->score,
            confidence_name(classify_confidence(t)),
            risk_name(t->risk), t->risk_why ? t->risk_why : "",
            dtype_name(t->dtype), dtier_name(t->dtype_tier), vsafe, msafe,
            (unsigned long)t->pkts, (unsigned long)t->pkt_rate,
            (unsigned long)(t->bytes_up/1024), (unsigned long)(t->bytes_down/1024),
            (unsigned long)t->evidence, (int)g_phase);
    }
    s_file.flush();
    spi_give();
}

// ===========================================================================
// UI  -  five screens
//
//   WELCOME  mode selection. Static, painted once.
//   SWEEP    the physical hunt. Dirty-field readout, peak-hold bar, dwell.
//   LIST     triage, highest risk first, three lines per row.
//   DETAIL   the evidence and the triage rule that fired.
//   SETUP    baseline, calibration, diagnostics.
//
// Every screen paints its chrome once on entry and then updates only fields
// whose content changed. Nothing is cleared per frame; see FLICKER-FREE
// FIELDS above for why that was the whole problem.
// ===========================================================================

#define TABBAR_Y 218
#define TAB_H     22
#define REDRAW_MS 200
#define LIST_ROWS  6
#define ROW_H     32
#define LIST_Y0   24          // first row's top edge
#define SCROLL_X  294         // left edge of the scroll column
#define SCROLL_W   26
#define ROW_CHARS  47         // row text width once the column is reserved
// Overlap one row when paging so the operator keeps their place in the list.
#define SCROLL_STEP (LIST_ROWS - 1)

static screen_t s_screen = SCR_WELCOME;
static uint8_t  s_screen_drawn = 0xFF;   // which screen's chrome is on glass
static uint8_t  s_sel_mac[6] = {0};
static rband_t  s_sel_band = BAND_24;
static bool     s_sel_valid = false;
static uint32_t s_last_draw = 0, s_last_touch = 0;
static uint16_t s_list_top = 0;
static bool     s_quiet = false;

static track_t *sel_track()
{
    if (!s_sel_valid) return NULL;
    return track_get(s_sel_mac, s_sel_band, false);
}

static void ui_set_screen(screen_t s)
{
    s_screen = s;
    s_screen_drawn = 0xFF;      // forces a chrome repaint on the next tick
    s_last_draw = 0;
}

static bool hit(const rect_t *r, uint16_t x, uint16_t y)
{
    return x >= r->x && x < r->x + r->w && y >= r->y && y < r->y + r->h;
}

static void draw_button(const rect_t *r, const char *label, const char *sub,
                        uint16_t frame, uint16_t label_col)
{
    draw_rect(r->x, r->y, r->w, r->h, frame);
    int lw = (int)strlen(label) * 12;
    draw_text_12((int16_t)(r->x + (r->w - lw) / 2),
                 (int16_t)(r->y + (sub ? 6 : (r->h - 16) / 2)), label,
                 label_col, C_BLACK);
    if (sub) {
        int sw = (int)strlen(sub) * 6;
        draw_text((int16_t)(r->x + (r->w - sw) / 2), (int16_t)(r->y + 26), sub,
                  C_DIM, C_BLACK);
    }
}

// ===========================================================================
// WELCOME  (static: painted once, never refreshed)
// ===========================================================================
static const rect_t BTN_ALL   = {  10,  56, 300, 44 };
static const rect_t BTN_WIFI  = {  10, 106, 145, 44 };
static const rect_t BTN_BLE   = { 165, 106, 145, 44 };
// TOUCH CAL used to sit beside WATCH here. It moved to SETUP alongside the
// other calibration, so WATCH takes the full width.
static const rect_t BTN_WATCH = {  10, 156, 300, 30 };

static void draw_welcome_chrome()
{
    fill_screen(C_BLACK);
    draw_text_12(8, 6, "VULPECULA", C_CYAN, C_BLACK);
    draw_text(122, 10, "BUG SWEEPER", C_DIM, C_BLACK);
    draw_hline(0, 26, SCR_W, C_PANEL);

    char b[56];
    snprintf(b, sizeof(b), "2.4GHz + %u x 5GHz%s   RSSI %s", (unsigned)s_n_ch5,
             s_dfs_ok ? " (DFS)" : "",
             g_cal.calibrated[BAND_24] ? "calibrated" : "UNCALIBRATED");
    draw_text(8, 34, b, s_n_ch5 ? (g_cal.calibrated[BAND_24] ? C_GREEN : C_AMBER)
                                : C_RED, C_BLACK);

    draw_button(&BTN_ALL,   "SWEEP ALL", "Wi-Fi pass, then BLE pass", C_CYAN,  C_CYAN);
    draw_button(&BTN_WIFI,  "WI-FI",     "2.4 + 5 GHz",              C_AMBER, C_AMBER);
    draw_button(&BTN_BLE,   "BLE",       "passive only",             C_AMBER, C_AMBER);
    draw_button(&BTN_WATCH, "WATCH", "stationary - round-robin all three",
                C_PANEL, C_DIM);

    draw_text(8, 194, "Sweep slowly. Hold at each fixture until the dwell", C_DIM, C_BLACK);
    draw_text(8, 204, "readout says covered, then tap to mark a position.", C_DIM, C_BLACK);
    draw_text(8, 224, "Radio-silent cameras are invisible. Check lenses.", C_AMBER, C_BLACK);
}

// ===========================================================================
// tab bar
// ===========================================================================
static const char *TAB_NAMES[SCR_TAB_COUNT] = { "HOME", "SWEEP", "LIST", "INFO", "SETUP" };

static void draw_tabbar()
{
    const int tw = SCR_W / SCR_TAB_COUNT;
    for (int i = 0; i < SCR_TAB_COUNT; i++) {
        int x = i * tw;
        bool act = (i == (int)s_screen);
        uint16_t bg = act ? C_CYAN : C_PANEL, fg = act ? C_BLACK : C_DIM;
        fill_rect(x, TABBAR_Y, tw - 1, TAB_H, bg);
        int lw = (int)strlen(TAB_NAMES[i]) * 6;
        draw_text(x + (tw - 1 - lw) / 2, TABBAR_Y + 7, TAB_NAMES[i], fg, bg);
    }
}

// ===========================================================================
// SWEEP
// ===========================================================================
static const rect_t BTN_NEXTPASS = { 214, 166, 100, 22 };

static field_t F_TIER, F_STATUS, F_PEAK, F_DIST, F_IDENT, F_RISK;
static field_t F_HELD, F_COVER, F_COUNTS, F_PROMPT, F_BASE;
static bar_t   B_PEAK;
static uint16_t s_tier_bg = 0xFFFF;

#define BAR_X 5
#define BAR_Y 74
#define BAR_W 310
#define BAR_H 18

static void draw_sweep_chrome()
{
    fill_screen(C_BLACK);
    draw_rect(BAR_X - 1, BAR_Y - 1, BAR_W + 2, BAR_H + 2, C_PANEL);
    draw_text(6,   BAR_Y + BAR_H + 4, "-90", C_DIM, C_BLACK);
    draw_text(296, BAR_Y + BAR_H + 4, "-10", C_DIM, C_BLACK);

    // Gate ticks are chrome: they only move when calibration changes.
    rband_t ab = (sched_active_proto() == BAND_BLE) ? BAND_BLE : BAND_24;
    draw_vline(BAR_X + ((g_cal.near_thresh[ab]    + 90) * BAR_W) / 80,
               BAR_Y - 4, BAR_H + 8, C_AMBER);
    draw_vline(BAR_X + ((g_cal.contact_thresh[ab] + 90) * BAR_W) / 80,
               BAR_Y - 4, BAR_H + 8, C_RED);

    field_init(&F_TIER,   6,   5, 8, 1, C_BLACK, C_GREEN);
    field_init(&F_STATUS, 200, 9, 19, 0, C_BLACK, C_GREEN);
    field_init(&F_PEAK,   6,  32, 9, 1, C_WHITE, C_BLACK);
    field_init(&F_DIST,   194, 32, 10, 1, C_DIM,  C_BLACK);
    field_init(&F_RISK,   6,  54, 52, 0, C_DIM,  C_BLACK);
    field_init(&F_IDENT,  6, 106, 52, 0, C_WHITE, C_BLACK);
    field_init(&F_HELD,   6, 120, 52, 0, C_DIM,  C_BLACK);
    field_init(&F_COVER,  6, 134, 52, 0, C_DIM,  C_BLACK);
    field_init(&F_COUNTS, 6, 148, 52, 0, C_DIM,  C_BLACK);
    field_init(&F_PROMPT, 6, 168, 33, 0, C_DIM,  C_BLACK);
    field_init(&F_BASE,   6, 196, 52, 0, C_CYAN, C_BLACK);
    bar_init(&B_PEAK, BAR_X, BAR_Y, BAR_W, BAR_H);
    s_tier_bg = 0xFFFF;
    draw_tabbar();
}

static void draw_sweep(uint32_t now)
{
    static track_t *list[64];
    uint16_t n = tracks_sorted(list, 64);
    track_t *top = n ? list[0] : NULL;

    int8_t peak = INT8_MIN;
    tier_t tier = TIER_AMBIENT;
    if (top) { peak = ring_peak(&top->ring, now, PEAK_WINDOW_MS); tier = top->tier; }
    sched_note_strongest(peak, now);

    if (tier == TIER_CONTACT)   led_rgb(50, 0, 0);
    else if (tier == TIER_NEAR) led_rgb(45, 22, 0);
    else                        led_rgb(0, 8, 0);

    uint16_t col = tier_colour(tier);
    char b[56];

    // Banner background is only repainted when the tier actually changes.
    if (col != s_tier_bg) {
        s_tier_bg = col;
        fill_rect(0, 0, SCR_W, 26, col);
        F_TIER.bg = col;   F_TIER.fg = C_BLACK;   field_invalidate(&F_TIER);
        F_STATUS.bg = col; F_STATUS.fg = C_BLACK; field_invalidate(&F_STATUS);
    }
    field_set(&F_TIER, tier_name(tier));
    field_set(&F_STATUS, sched_status());

    if (peak == INT8_MIN) snprintf(b, sizeof(b), "  -- dBm");
    else                  snprintf(b, sizeof(b), "%4d dBm", (int)peak);
    field_set(&F_PEAK, b);

    if (top && top->dist_m >= 0.0f) snprintf(b, sizeof(b), "~%.2f m", top->dist_m);
    else                            snprintf(b, sizeof(b), "no TX ref");
    field_set(&F_DIST, b);

    int fill = (peak == INT8_MIN) ? 0
             : ((((peak < -90 ? -90 : (peak > -10 ? -10 : peak)) + 90) * BAR_W) / 80);
    bar_set(&B_PEAK, (int16_t)fill, col);

    // Triage verdict for the strongest track, with the rule that produced it.
    if (top) {
        snprintf(b, sizeof(b), "%-4s %-8s %.34s", risk_name(top->risk),
                 dtype_name(top->dtype), top->risk_why ? top->risk_why : "");
        field_set_col(&F_RISK, b, risk_colour(top->risk) == C_PANEL
                                  ? C_DIM : risk_colour(top->risk));
    } else {
        field_set_col(&F_RISK, "", C_DIM);
    }

    if (top) {
        const char *who = top->vendor[0] ? top->vendor
                        : (top->name[0] ? top->name : "(no name)");
        snprintf(b, sizeof(b), "%-4s ch%-3u %.40s", band_name(top->band),
                 (unsigned)top->channel, who);
        field_set_col(&F_IDENT, b, C_WHITE);
    } else {
        field_set_col(&F_IDENT, "nothing above ambient", C_DIM);
    }

    uint32_t rms = sched_position_radio_ms(), cov = sched_covered_interval_ms();
    snprintf(b, sizeof(b), "held %.1fs radio at this position", rms / 1000.0f);
    field_set(&F_HELD, b);
    if (!cov) snprintf(b, sizeof(b), "covers: nothing yet");
    else      snprintf(b, sizeof(b), "covers transmit intervals <= %.2fs", cov / 1000.0f);
    field_set_col(&F_COVER, b, cov >= 1000 ? C_GREEN : C_AMBER);
    snprintf(b, sizeof(b), "tracks %u   positions %u   ident %lu",
             (unsigned)g_used, (unsigned)s_positions, (unsigned long)s_ie_seen);
    field_set(&F_COUNTS, b);

    if (s_pass_prompt) {
        field_set_col(&F_PROMPT, "PASS 2/2 - WALK THE ROOM AGAIN", C_CYAN);
    } else if (sched_sweep_too_fast()) {
        field_set_col(&F_PROMPT, "SWEEPING TOO FAST - hold still", C_YELLOW);
    } else {
        field_set_col(&F_PROMPT, "tap body to mark a position", C_DIM);
        if (s_mode == MODE_SWEEP_ALL && s_all_phase == 0) {
            draw_rect(BTN_NEXTPASS.x, BTN_NEXTPASS.y, BTN_NEXTPASS.w,
                      BTN_NEXTPASS.h, C_CYAN);
            draw_text(BTN_NEXTPASS.x + 14, BTN_NEXTPASS.y + 7, "BLE PASS >",
                      C_CYAN, C_BLACK);
        }
    }

    if (baseline_phase() != PHASE_FREE) {
        uint32_t el = baseline_elapsed_ms(now);
        uint32_t left = el >= BASELINE_CAPTURE_MS ? 0 : (BASELINE_CAPTURE_MS - el) / 1000;
        snprintf(b, sizeof(b), "BASELINE %s  %lus remaining",
                 baseline_phase() == PHASE_CORRIDOR ? "CORRIDOR" : "ROOM",
                 (unsigned long)left);
        field_set_col(&F_BASE, b, left ? C_CYAN : C_GREEN);
    } else {
        field_set(&F_BASE, "");
    }
}

// ===========================================================================
// LIST  -  six rows, three lines each, highest risk first
//
//   RISK  TYPE      name or SSID
//         vendor / model
//         band ch  peak  dist  evidence letters
// ===========================================================================
static const rect_t BTN_LUP = { SCROLL_X, LIST_Y0,      SCROLL_W, 92 };
static const rect_t BTN_LDN = { SCROLL_X, LIST_Y0 + 96,  SCROLL_W, 92 };

static field_t F_LHDR;
static field_t F_ROW[LIST_ROWS][3];
static uint16_t s_row_stripe[LIST_ROWS];
// Re-sorting at the redraw rate makes rows churn even with zero flicker, so
// the order is refreshed on its own slower cadence.
#define LIST_SORT_MS 1000
static uint32_t s_last_sort = 0;
static track_t *s_lview[MAX_TRACKS];
static uint16_t s_lview_n = 0;

// Solid triangles read better than a caret glyph at this size.
static void draw_tri(int16_t cx, int16_t cy, int16_t half, bool up, uint16_t c)
{
    for (int16_t i = 0; i < half; i++) {
        int16_t w = (int16_t)(2 * (half - i) - 1);
        int16_t y = up ? (int16_t)(cy + i) : (int16_t)(cy - i);
        fill_rect((int16_t)(cx - (half - i) + 1), y, w, 1, c);
    }
}

static void draw_scroll_col(uint16_t n)
{
    bool can_up = (s_list_top > 0);
    bool can_dn = (n > LIST_ROWS) && (s_list_top + LIST_ROWS < n);

    fill_rect(BTN_LUP.x, BTN_LUP.y, BTN_LUP.w, BTN_LUP.h, C_BLACK);
    fill_rect(BTN_LDN.x, BTN_LDN.y, BTN_LDN.w, BTN_LDN.h, C_BLACK);
    draw_rect(BTN_LUP.x, BTN_LUP.y, BTN_LUP.w, BTN_LUP.h,
              can_up ? C_CYAN : C_PANEL);
    draw_rect(BTN_LDN.x, BTN_LDN.y, BTN_LDN.w, BTN_LDN.h,
              can_dn ? C_CYAN : C_PANEL);
    draw_tri((int16_t)(BTN_LUP.x + BTN_LUP.w / 2),
             (int16_t)(BTN_LUP.y + 36), 8, true,  can_up ? C_CYAN : C_PANEL);
    draw_tri((int16_t)(BTN_LDN.x + BTN_LDN.w / 2),
             (int16_t)(BTN_LDN.y + 56), 8, false, can_dn ? C_CYAN : C_PANEL);

    // Position thumb: shows how far down a long list you are, which the
    // "7-12 of 34" header alone does not convey at a glance.
    int16_t tx = (int16_t)(BTN_LUP.x + 2), tw = (int16_t)(SCROLL_W - 4);
    int16_t ty = (int16_t)(BTN_LUP.y + 76), th = 16;
    fill_rect(tx, ty, tw, th, C_BLACK);
    if (n > LIST_ROWS) {
        int16_t pos = (int16_t)((s_list_top * (tw - 4)) / (n - LIST_ROWS));
        fill_rect(tx, (int16_t)(ty + 6), tw, 2, C_PANEL);
        fill_rect((int16_t)(tx + pos), (int16_t)(ty + 4), 4, 6, C_CYAN);
    }
}

static void draw_list_chrome()
{
    fill_screen(C_BLACK);
    fill_rect(0, 0, SCR_W, 20, C_PANEL);
    field_init(&F_LHDR, 4, 3, 52, 0, C_CYAN, C_PANEL);
    field_invalidate(&F_LHDR);
    draw_text(4, 12, "S STREAM R ROOM-ONLY C CONTACT A CAM-AP N NAME",
              C_DIM, C_PANEL);
    for (int r = 0; r < LIST_ROWS; r++) {
        int y = LIST_Y0 + r * ROW_H;
        field_init(&F_ROW[r][0], 8,  y,      ROW_CHARS, 0, C_WHITE, C_BLACK);
        field_init(&F_ROW[r][1], 8,  y + 10, ROW_CHARS, 0, C_DIM,   C_BLACK);
        field_init(&F_ROW[r][2], 8,  y + 20, ROW_CHARS, 0, C_DIM,   C_BLACK);
        s_row_stripe[r] = 0xFFFF;
    }
    draw_tabbar();
}

static void draw_list(uint32_t now)
{
    if (s_last_sort == 0 || (uint32_t)(now - s_last_sort) >= LIST_SORT_MS) {
        s_last_sort = now;
        s_lview_n = tracks_sorted(s_lview, MAX_TRACKS);
    }
    uint16_t n = s_lview_n;
    char b[60];

    snprintf(b, sizeof(b), "%u tracks   %u-%u shown   highest risk first",
             (unsigned)n, (unsigned)(n ? s_list_top + 1 : 0),
             (unsigned)(s_list_top + LIST_ROWS < n ? s_list_top + LIST_ROWS : n));
    field_set(&F_LHDR, b);

    // Clamp so the last page is full rather than leaving blank rows below a
    // partially scrolled list.
    uint16_t max_top = (n > LIST_ROWS) ? (uint16_t)(n - LIST_ROWS) : 0;
    if (s_list_top > max_top) s_list_top = max_top;

    static uint16_t drawn_top = 0xFFFF, drawn_n = 0xFFFF;
    if (s_list_top != drawn_top || n != drawn_n) {
        drawn_top = s_list_top; drawn_n = n;
        draw_scroll_col(n);
    }

    for (int r = 0; r < LIST_ROWS; r++) {
        uint16_t idx = (uint16_t)(s_list_top + r);
        int y = LIST_Y0 + r * ROW_H;

        if (idx >= n) {
            if (s_row_stripe[r] != C_BLACK) {
                fill_rect(0, y, 4, ROW_H - 4, C_BLACK);
                s_row_stripe[r] = C_BLACK;
            }
            field_set(&F_ROW[r][0], "");
            field_set(&F_ROW[r][1], "");
            field_set(&F_ROW[r][2], "");
            continue;
        }

        track_t *t = s_lview[idx];
        uint16_t rc = risk_colour(t->risk);
        if (rc != s_row_stripe[r]) {
            fill_rect(0, y, 4, ROW_H - 4, rc);
            s_row_stripe[r] = rc;
        }

        // line 1: risk + device type + name
        snprintf(b, sizeof(b), "%-4s %-8s %.36s", risk_name(t->risk),
                 dtype_name(t->dtype),
                 t->name[0] ? t->name : "(unnamed)");
        field_set_col(&F_ROW[r][0], b, rc == C_PANEL ? C_DIM : rc);

        // line 2: vendor and model, with where the vendor came from
        if (t->vendor[0] || t->model[0])
            snprintf(b, sizeof(b), "  %.20s %.20s%s", t->vendor,
                     t->model, t->vendor_tier == DT_TIER_A ? " *" : "");
        else
            snprintf(b, sizeof(b), "  vendor unknown");
        field_set_col(&F_ROW[r][1], b, C_WHITE);

        // line 3: signal, distance and evidence letters
        char ev[8]; uint8_t k = 0;
        if (t->evidence & E_STREAM_PROFILE)  ev[k++] = 'S';
        if (t->evidence & E_ROOM_ONLY)       ev[k++] = 'R';
        if (t->evidence & E_CONTACT_TIER)    ev[k++] = 'C';
        if (t->evidence & E_SOFTAP_CAM_SSID) ev[k++] = 'A';
        if (t->evidence & (E_BLE_CAM_NAME | E_BLE_MIC_NAME)) ev[k++] = 'N';
        ev[k] = 0;
        if (t->dist_m >= 0.0f)
            snprintf(b, sizeof(b), "  %-4s ch%-3u %4ddBm ~%.2fm  %s",
                     band_name(t->band), (unsigned)t->channel,
                     (int)t->peak_session, t->dist_m, ev);
        else
            snprintf(b, sizeof(b), "  %-4s ch%-3u %4ddBm  %-8s %s",
                     band_name(t->band), (unsigned)t->channel,
                     (int)t->peak_session, dtier_name(t->dtype_tier), ev);
        field_set_col(&F_ROW[r][2], b, C_DIM);
    }
}

// ===========================================================================
// DETAIL
// ===========================================================================
static void draw_detail_chrome()
{
    fill_screen(C_BLACK);
    draw_tabbar();
}

static void draw_detail(uint32_t now)
{
    (void)now;
    static char shown[12][44];
    static uint8_t shown_n = 0;
    // Keyed on the FULL identity. The previous version compared only
    // mac[5], so two tracks sharing a last byte would show each other's
    // header, name included.
    static uint8_t  last_mac[6] = {0};
    static rband_t  last_band = BAND_COUNT;
    static bool     last_valid = false;

    track_t *t = sel_track();
    if (!t || !t->used) {
        if (shown_n != 0xFE) {
            fill_rect(0, 0, SCR_W, TABBAR_Y, C_BLACK);
            draw_text_12(8, 14, s_sel_valid ? "TRACK LOST" : "NO SELECTION",
                         C_DIM, C_BLACK);
            draw_text(8, 40, s_sel_valid
                      ? "That device is no longer being tracked."
                      : "Tap a row on the LIST screen.", C_DIM, C_BLACK);
            shown_n = 0xFE;
        }
        return;
    }

    bool changed = !last_valid || last_band != t->band ||
                   memcmp(last_mac, t->mac, 6) != 0 || shown_n == 0xFE;
    if (changed) {
        memcpy(last_mac, t->mac, 6);
        last_band  = t->band;
        last_valid = true;
        shown_n = 0;
        memset(shown, 0, sizeof(shown));
        fill_rect(0, 0, SCR_W, TABBAR_Y, C_BLACK);

        uint16_t rc = risk_colour(t->risk);
        uint16_t hb = (rc == C_PANEL) ? C_PANEL : rc;
        fill_rect(0, 0, SCR_W, 18, hb);
        char h[56];
        snprintf(h, sizeof(h), "%-4s  %s", risk_name(t->risk), dtype_name(t->dtype));
        draw_text(4, 5, h, C_BLACK, hb);

        // The device name gets the full width of its own line. Sharing a line
        // with the risk badge and the type meant a long SSID was the thing
        // that got truncated, which is backwards: the name is what the
        // operator reads out, writes down and searches for.
        char nm[56];
        snprintf(nm, sizeof(nm), "NAME %s",
                 t->name[0] ? t->name : "(none advertised)");
        draw_text(4, 22, nm, t->name[0] ? C_WHITE : C_DIM, C_BLACK);

        char pseudo[20], sub[60];
        log_pseudonym(t->mac, pseudo, sizeof(pseudo));
        snprintf(sub, sizeof(sub), "%s CH%u  ID %s  %lu PKTS", band_name(t->band),
                 (unsigned)t->channel, pseudo, (unsigned long)t->pkts);
        draw_text(4, 34, sub, C_DIM, C_BLACK);
    }

    // Only rewrite evidence lines whose text changed.
    // 12 lines at 14px from y=48 ends at 216, one pixel clear of the tab bar.
    char lines[12][44];
    uint8_t nl = classify_explain(t, lines, 12);
    for (uint8_t i = 0; i < 12; i++) {
        const char *want = (i < nl) ? lines[i] : "";
        if (strncmp(shown[i], want, 44) == 0) continue;
        char pad[48];
        snprintf(pad, sizeof(pad), "%-44s", want);
        uint16_t c = want[0] == '+' ? C_WHITE
                   : want[0] == '~' ? C_YELLOW : C_CYAN;
        draw_text(6, 48 + i * 14, pad, c, C_BLACK);
        strncpy(shown[i], want, 43); shown[i][43] = 0;
    }
    shown_n = nl;
}

// ===========================================================================
// PROCEDURE PAGES
//
// Baseline capture and RSSI calibration are both multi-step procedures with a
// right and a wrong way to run them, and neither is inferable from a button
// label. Run either one wrong and the result is worse than not running it:
// a baseline taken without leaving the room flags nothing, and a calibration
// taken with the reference in the wrong place moves the gate thresholds to
// values the operator then trusts.
//
// So each is fronted by a page that says what it does, what it needs, and how
// to run it, with START and BACK at the bottom. Nothing begins until START.
// ===========================================================================

static const rect_t BTN_INFO_BACK  = {   8, 200, 145, 32 };
static const rect_t BTN_INFO_START = { 167, 200, 145, 32 };

static field_t F_INFO_STAT, F_INFO_BTN;

static void draw_info_page(const infopage_t *p)
{
    fill_screen(C_BLACK);
    draw_text_12(6, 4, p->title, C_CYAN, C_BLACK);
    draw_hline(0, 23, SCR_W, C_PANEL);

    for (int i = 0; i < INFO_MAX_LINES && p->lines[i]; i++) {
        const char *l = p->lines[i];
        // A leading '!' marks a warning line, '>' a numbered step.
        uint16_t c = (l[0] == '!') ? C_AMBER : (l[0] == '>') ? C_WHITE : C_DIM;
        draw_text(6, 28 + i * 12, (l[0] == '!' || l[0] == '>') ? l + 1 : l,
                  c, C_BLACK);
    }

    draw_rect(BTN_INFO_BACK.x, BTN_INFO_BACK.y, BTN_INFO_BACK.w,
              BTN_INFO_BACK.h, C_PANEL);
    draw_text_12(BTN_INFO_BACK.x + 43, BTN_INFO_BACK.y + 8, "BACK",
                 C_DIM, C_BLACK);

    draw_rect(BTN_INFO_START.x, BTN_INFO_START.y, BTN_INFO_START.w,
              BTN_INFO_START.h, C_CYAN);

    // Live status sits just above the buttons: sample count while
    // calibrating, phase countdown while baselining.
    field_init(&F_INFO_STAT, 6, 174, 51, 0, C_AMBER, C_BLACK);
    field_init(&F_INFO_BTN, BTN_INFO_START.x + 8, BTN_INFO_START.y + 8,
               10, 1, C_CYAN, C_BLACK);
}

// ---------------------------------------------------------------------------
// Baseline capture
// ---------------------------------------------------------------------------
static const infopage_t INFO_BASELINE = {
    "BASELINE CAPTURE",
    {
        "Records what is audible in the corridor, then in",
        "the room, and flags devices heard ONLY inside.",
        "",
        "The 2m gate is the crude first pass: a neighbour's",
        "camera against the party wall can still cross it.",
        "Only the corridor-vs-room difference separates",
        "their hardware from yours.",
        "",
        "> 1. Stand OUTSIDE the door, then tap START.",
        "> 2. Wait 60s. Go inside, then tap ROOM >.",
        "> 3. Wait 60s, then tap APPLY.",
        "!Seen only inside = flagged R, and ranked up.",
        NULL
    }
};

// ---------------------------------------------------------------------------
// RSSI calibration
// ---------------------------------------------------------------------------
static const infopage_t INFO_RSSICAL = {
    "RSSI CALIBRATION",
    {
        "Measures what THIS board reports at 2.00m and at",
        "0.30m, so the gate thresholds are measured, not",
        "calculated. Shipped values are path-loss maths",
        "plus an assumed margin; antenna, enclosure and",
        "board variation move them several dB.",
        "",
        "Needs the reference beacon on a spare ESP32, a",
        "tape measure, and a quiet room.",
        "> 1. Board flat at chest height. Do NOT hold it.",
        "> 2. Ref at 2.00m, same height. START, stand clear.",
        "> 3. At 40+ samples move ref to 0.30m, tap 0.30M >.",
        "!Fails if the two readings are under 8dB apart.",
        NULL
    }
};

// The action button names the next step, so a tap is never a guess.
static const char *info_action_label(const infopage_t *p)
{
    if (p == &INFO_BASELINE) {
        switch (baseline_phase()) {
            case PHASE_CORRIDOR: return "ROOM >";
            case PHASE_ROOM:     return "APPLY";
            default:             return "START";
        }
    }
    switch (s_cal) {
        case CAL_NEAR:    return "0.30M >";
        case CAL_CONTACT: return "FINISH";
        case CAL_DONE:    return "COMMIT";
        case CAL_FAILED:  return "RETRY";
        default:          return "START";
    }
}

// Refreshed every tick while a procedure page is open.
static void draw_info_live(const infopage_t *p, uint32_t now)
{
    char b[56];

    const char *lab = info_action_label(p);
    int pad = (10 - (int)strlen(lab)) / 2;
    snprintf(b, sizeof(b), "%*s%s", pad > 0 ? pad : 0, "", lab);
    field_set(&F_INFO_BTN, b);

    if (p == &INFO_BASELINE) {
        if (baseline_phase() == PHASE_FREE) {
            field_set_col(&F_INFO_STAT, "Not started. Step out of the room first.",
                          C_DIM);
        } else {
            uint32_t el = baseline_elapsed_ms(now);
            uint32_t left = el >= BASELINE_CAPTURE_MS ? 0
                          : (BASELINE_CAPTURE_MS - el) / 1000;
            snprintf(b, sizeof(b), "%s CAPTURE - %lus left - %u tracks",
                     baseline_phase() == PHASE_CORRIDOR ? "CORRIDOR" : "ROOM",
                     (unsigned long)left, (unsigned)g_used);
            field_set_col(&F_INFO_STAT, b, left ? C_CYAN : C_GREEN);
        }
        return;
    }

    switch (s_cal) {
        case CAL_IDLE:
            snprintf(b, sizeof(b), "Ready - will calibrate the %s band",
                     band_name(sched_active_proto()));
            field_set_col(&F_INFO_STAT, b, C_AMBER);
            break;
        case CAL_DONE:
            snprintf(b, sizeof(b), "MEASURED near %d  contact %d dBm",
                     (int)s_cal_near, (int)s_cal_contact);
            field_set_col(&F_INFO_STAT, b, C_GREEN);
            break;
        case CAL_FAILED:
            field_set_col(&F_INFO_STAT,
                          "FAILED - ref never moved, or no antenna fitted",
                          C_RED);
            break;
        default:
            snprintf(b, sizeof(b), "%s  %u samples", cal_prompt(),
                     (unsigned)s_cal_n);
            field_set_col(&F_INFO_STAT, b, s_cal_n >= 40 ? C_GREEN : C_AMBER);
            break;
    }
}

// ===========================================================================
// SETUP
// ===========================================================================
static const rect_t BTN_PHASE = {   8,  38, 145, 28 };
static const rect_t BTN_CAL   = { 167,  38, 145, 28 };
static const rect_t BTN_TCAL2 = {   8, 156, 145, 26 };
static const rect_t BTN_RESET = { 167, 156, 145, 26 };
static const rect_t BTN_QUIET = {   8, 186, 145, 26 };
static const rect_t BTN_STOP  = { 167, 186, 145, 26 };

static field_t F_PHASEB, F_CALB, F_S5, F_SWIFI, F_SBLE, F_SSD, F_SGATE,
               F_SCAL, F_STOUCH, F_SQUIET;

static void draw_setup_chrome()
{
    fill_screen(C_BLACK);
    draw_text_12(6, 4, "SETUP", C_CYAN, C_BLACK);
    draw_rect(BTN_PHASE.x, BTN_PHASE.y, BTN_PHASE.w, BTN_PHASE.h, C_CYAN);
    draw_rect(BTN_CAL.x,   BTN_CAL.y,   BTN_CAL.w,   BTN_CAL.h,   C_AMBER);
    draw_rect(BTN_TCAL2.x, BTN_TCAL2.y, BTN_TCAL2.w, BTN_TCAL2.h, C_PANEL);
    draw_rect(BTN_RESET.x, BTN_RESET.y, BTN_RESET.w, BTN_RESET.h, C_PANEL);
    draw_rect(BTN_QUIET.x, BTN_QUIET.y, BTN_QUIET.w, BTN_QUIET.h, C_PANEL);
    draw_rect(BTN_STOP.x,  BTN_STOP.y,  BTN_STOP.w,  BTN_STOP.h,  C_PANEL);
    draw_text(BTN_TCAL2.x + 38, BTN_TCAL2.y + 9, "TOUCH CAL",   C_DIM, C_BLACK);
    draw_text(BTN_RESET.x + 30, BTN_RESET.y + 9, "NEW SESSION", C_DIM, C_BLACK);
    draw_text(BTN_STOP.x  + 30, BTN_STOP.y  + 9, "STOP RADIOS", C_DIM, C_BLACK);

    field_init(&F_PHASEB, BTN_PHASE.x + 8, BTN_PHASE.y + 10, 22, 0, C_CYAN,  C_BLACK);
    field_init(&F_CALB,   BTN_CAL.x   + 4, BTN_CAL.y   + 4,  23, 0, C_AMBER, C_BLACK);
    field_init(&F_SCAL,   BTN_CAL.x   + 4, BTN_CAL.y   + 16, 23, 0, C_DIM,   C_BLACK);
    field_init(&F_S5,     8,  74, 52, 0, C_GREEN, C_BLACK);
    field_init(&F_SWIFI,  8,  86, 52, 0, C_WHITE, C_BLACK);
    field_init(&F_SBLE,   8,  98, 52, 0, C_WHITE, C_BLACK);
    field_init(&F_SSD,    8, 110, 52, 0, C_GREEN, C_BLACK);
    field_init(&F_SGATE,  8, 124, 52, 0, C_WHITE, C_BLACK);
    field_init(&F_STOUCH, 8, 138, 52, 0, C_DIM,   C_BLACK);
    field_init(&F_SQUIET, BTN_QUIET.x + 16, BTN_QUIET.y + 9, 20, 0, C_DIM, C_BLACK);
    draw_tabbar();
}

static void draw_setup(uint32_t now)
{
    (void)now;
    char b[60];

    field_set(&F_PHASEB, baseline_phase() == PHASE_CORRIDOR ? "PHASE: CORRIDOR"
                       : baseline_phase() == PHASE_ROOM     ? "PHASE: ROOM"
                                                            : "BASELINE: START");
    field_set(&F_CALB, (s_cal == CAL_IDLE) ? "RSSI CAL" : cal_prompt());
    if (s_cal == CAL_NEAR || s_cal == CAL_CONTACT) {
        char cb[32];
        snprintf(cb, sizeof(cb), "%u/40 samples", (unsigned)s_cal_n);
        field_set_col(&F_SCAL, cb, s_cal_n >= 40 ? C_GREEN : C_AMBER);
    } else {
        field_set_col(&F_SCAL, cal_prompt(), s_cal == CAL_FAILED ? C_RED : C_DIM);
    }

    snprintf(b, sizeof(b), "5GHz usable %u ch %s", (unsigned)s_n_ch5,
             s_n_ch5 == 0 ? "NONE" : (s_dfs_ok ? "incl DFS" : "no DFS 52-144"));
    field_set_col(&F_S5, b, s_n_ch5 ? (s_dfs_ok ? C_GREEN : C_AMBER) : C_RED);

    snprintf(b, sizeof(b), "WiFi frames %lu drop %lu  ident %lu drop %lu",
             (unsigned long)s_frames, (unsigned long)s_frames_drop,
             (unsigned long)s_ie_seen, (unsigned long)s_ie_drop);
    field_set(&F_SWIFI, b);

    snprintf(b, sizeof(b), "BLE adverts %lu  with TX ref %lu",
             (unsigned long)s_adv, (unsigned long)s_with_ref);
    field_set(&F_SBLE, b);

    snprintf(b, sizeof(b), "SD %s", s_sd_ok ? s_fname : "not mounted");
    field_set_col(&F_SSD, b, s_sd_ok ? C_GREEN : C_AMBER);

    snprintf(b, sizeof(b), "gate N/C 2.4 %d/%d  5 %d/%d  BLE %d/%d",
             g_cal.near_thresh[BAND_24], g_cal.contact_thresh[BAND_24],
             g_cal.near_thresh[BAND_5],  g_cal.contact_thresh[BAND_5],
             g_cal.near_thresh[BAND_BLE],g_cal.contact_thresh[BAND_BLE]);
    field_set(&F_SGATE, b);

    snprintf(b, sizeof(b), "touch %s  raw x%u y%u z%u  RSSI %s",
             g_tcal.valid ? "cal" : "DEFAULT",
             (unsigned)g_touch_raw_x, (unsigned)g_touch_raw_y,
             (unsigned)g_touch_raw_z,
             (g_cal.calibrated[BAND_24] || g_cal.calibrated[BAND_BLE])
                ? "cal" : "DEFAULT");
    field_set_col(&F_STOUCH, b, g_tcal.valid ? C_DIM : C_AMBER);

    field_set(&F_SQUIET, s_quiet ? "SCREEN: OFF" : "SCREEN: ON");
}

// ===========================================================================
// touch routing
// ===========================================================================
static void handle_touch(uint16_t x, uint16_t y, uint32_t now)
{
    if (s_screen == SCR_WELCOME) {
        if      (hit(&BTN_ALL,  x, y)) { sched_set_mode(MODE_SWEEP_ALL);  ui_set_screen(SCR_SWEEP); }
        else if (hit(&BTN_WIFI, x, y)) { sched_set_mode(MODE_WIFI_SWEEP); ui_set_screen(SCR_SWEEP); }
        else if (hit(&BTN_BLE,  x, y)) { sched_set_mode(MODE_BLE_SWEEP);  ui_set_screen(SCR_SWEEP); }
        else if (hit(&BTN_WATCH,x, y)) { sched_set_mode(MODE_WATCH);      ui_set_screen(SCR_SWEEP); }
        return;
    }

    // Procedure pages have no tab bar: BACK and START are the only exits, so
    // a stray tap cannot abandon a half-read explanation.
    if (s_screen == SCR_BASELINE_INFO || s_screen == SCR_RSSICAL_INFO) {
        if (hit(&BTN_INFO_BACK, x, y)) { ui_set_screen(SCR_SETUP); return; }
        if (hit(&BTN_INFO_START, x, y)) {
            if (s_screen == SCR_BASELINE_INFO) {
                phase_t ph = baseline_phase();
                if (ph == PHASE_FREE) {
                    baseline_set_phase(PHASE_CORRIDOR, now);
                    // Straight to SWEEP: the countdown lives there and the
                    // operator is about to walk out of the room.
                    ui_set_screen(SCR_SWEEP);
                } else if (ph == PHASE_CORRIDOR) {
                    baseline_set_phase(PHASE_ROOM, now);
                    ui_set_screen(SCR_SWEEP);
                } else {
                    baseline_apply_diff();
                    baseline_set_phase(PHASE_FREE, now);
                    log_snapshot(now, "BASELINE");
                    // The diff is the whole payoff, so land on the list where
                    // the newly promoted devices have sorted to the top.
                    s_list_top = 0;
                    s_last_sort = 0;
                    ui_set_screen(SCR_LIST);
                }
            } else {
                cal_advance();
                // Committing returns to CAL_IDLE; anything else keeps the
                // page open so the sample count stays visible.
                if (s_cal == CAL_IDLE) ui_set_screen(SCR_SETUP);
                else                   s_screen_drawn = 0xFF;   // relabel
            }
        }
        return;
    }

    if (y >= TABBAR_Y) {
        int tw = SCR_W / SCR_TAB_COUNT;
        int tab = x / tw;
        if (tab >= 0 && tab < SCR_TAB_COUNT) ui_set_screen((screen_t)tab);
        return;
    }

    if (s_screen == SCR_SWEEP) {
        if (s_pass_prompt) { s_pass_prompt = false; return; }
        if (s_mode == MODE_SWEEP_ALL && s_all_phase == 0 &&
            hit(&BTN_NEXTPASS, x, y)) { sched_all_next_pass(now); ui_set_screen(SCR_SWEEP); return; }
        // Anywhere else in the body marks a position. Deliberately a huge
        // target: you are holding this at arm's length behind a headboard.
        sched_mark_position(now);
    } else if (s_screen == SCR_LIST) {
        uint16_t max_top = (s_lview_n > LIST_ROWS)
                         ? (uint16_t)(s_lview_n - LIST_ROWS) : 0;

        if (hit(&BTN_LUP, x, y)) {
            s_list_top = (s_list_top > SCROLL_STEP)
                       ? (uint16_t)(s_list_top - SCROLL_STEP) : 0;
            return;
        }
        if (hit(&BTN_LDN, x, y)) {
            s_list_top = (uint16_t)(s_list_top + SCROLL_STEP);
            if (s_list_top > max_top) s_list_top = max_top;
            return;
        }
        // Header: jump back to the top of the list.
        if (y < LIST_Y0) { s_list_top = 0; return; }

        // Every row is now selectable - the last one is no longer stolen for
        // paging, which is what the scroll column is for.
        int row = (y - LIST_Y0) / ROW_H;
        if (row >= LIST_ROWS) row = LIST_ROWS - 1;
        uint16_t idx = (uint16_t)(s_list_top + row);
        if (idx < s_lview_n) {
            memcpy(s_sel_mac, s_lview[idx]->mac, 6);
            s_sel_band  = s_lview[idx]->band;
            s_sel_valid = true;
            ui_set_screen(SCR_DETAIL);
        }
    } else if (s_screen == SCR_SETUP) {
        // Both open their procedure page, at every stage rather than only
        // from a standing start: mid-procedure is exactly when it matters
        // what the next tap will do, and the page shows live progress.
        if (hit(&BTN_PHASE, x, y)) {
            ui_set_screen(SCR_BASELINE_INFO);
            return;
        } else if (hit(&BTN_CAL, x, y)) {
            ui_set_screen(SCR_RSSICAL_INFO);
            return;
        } else if (hit(&BTN_TCAL2, x, y)) {
            if (!touch_calibrate_interactive()) touch_calibrate_interactive();
            ui_set_screen(SCR_SETUP);
        } else if (hit(&BTN_RESET, x, y)) {
            tracks_reset();
            s_sel_valid = false;
            s_positions = 0;
            s_lview_n = 0; s_last_sort = 0;
            sched_mark_position(now);
        } else if (hit(&BTN_QUIET, x, y)) {
            s_quiet = true;
            digitalWrite(PIN_BL, LOW);
        } else if (hit(&BTN_STOP, x, y)) {
            sched_set_mode(MODE_IDLE);
            ui_set_screen(SCR_WELCOME);
        }
    }
}

static void ui_tick(uint32_t now)
{
    uint16_t tx, ty;
    if (touch_read(&tx, &ty) && (uint32_t)(now - s_last_touch) > 260) {
        s_last_touch = now;
        // A touch always wakes the screen: quiet mode must never be a state
        // the operator cannot get out of.
        if (s_quiet) { s_quiet = false; digitalWrite(PIN_BL, HIGH); ui_set_screen(s_screen); }
        else handle_touch(tx, ty, now);
    }
    if (s_quiet) return;                 // backlight off: no SPI traffic

    // Chrome first, once per screen entry.
    if (s_screen_drawn != (uint8_t)s_screen) {
        switch (s_screen) {
            case SCR_WELCOME: draw_welcome_chrome(); break;
            case SCR_SWEEP:   draw_sweep_chrome();   break;
            case SCR_LIST:    draw_list_chrome();    break;
            case SCR_DETAIL:  draw_detail_chrome();  break;
            case SCR_BASELINE_INFO: draw_info_page(&INFO_BASELINE); break;
            case SCR_RSSICAL_INFO:  draw_info_page(&INFO_RSSICAL);  break;
            default:          draw_setup_chrome();   break;
        }
        s_screen_drawn = (uint8_t)s_screen;
        s_last_draw = 0;
    }

    if (s_last_draw && (uint32_t)(now - s_last_draw) < REDRAW_MS) return;
    s_last_draw = now;

    switch (s_screen) {
        case SCR_WELCOME:                 // fully static, nothing to refresh
        case SCR_BASELINE_INFO: draw_info_live(&INFO_BASELINE, now); break;
        case SCR_RSSICAL_INFO:  draw_info_live(&INFO_RSSICAL,  now); break;
        case SCR_SWEEP:   draw_sweep(now);  break;
        case SCR_LIST:    draw_list(now);   break;
        case SCR_DETAIL:  draw_detail(now); break;
        default:          draw_setup(now);  break;
    }
}

// ===========================================================================
// TASKS
//
// The radio pump and the UI have very different latency needs: dropping a
// captured frame loses evidence, dropping a frame of UI loses nothing. So the
// pump runs at higher priority on its own task and the UI rides the Arduino
// loop.
// ===========================================================================

static void pump_task(void *arg)
{
    (void)arg;
    uint32_t last_tick = 0, last_log = 0;
    for (;;) {
        uint32_t now = millis();
        sched_tick(now);
        wifi_cap_pump(now);
        ble_cap_pump(now);
        ie_pump(now);          // identity parsing, a few frames per tick

        // 5 Hz. Per-packet would be wasted work; slower and the sweep meter
        // lags the operator's hand.
        if ((uint32_t)(now - last_tick) >= 200) {
            last_tick = now;
            tracks_tick(now);

            if (s_cal == CAL_NEAR || s_cal == CAL_CONTACT) {
                static track_t *l[16];
                uint16_t n = tracks_sorted(l, 16);
                for (uint16_t i = 0; i < n; i++)
                    if (l[i]->band == s_cal_band) {
                        cal_feed(ring_peak(&l[i]->ring, now, PEAK_WINDOW_MS));
                        break;
                    }
            }

            for (uint16_t i = 0; i < MAX_TRACKS; i++) {
                track_t *t = &g_tracks[i];
                if (!t->used) continue;
                if (t->tier >= TIER_NEAR && !t->alerted && t->score >= SCORE_POSSIBLE) {
                    t->alerted = true;
                    log_snapshot(now, "ALERT");
                    char ps[20]; log_pseudonym(t->mac, ps, sizeof(ps));
                    Serial.printf("[%s] %s %s %s %ddBm %s/%s %.16s %.16s\n",
                                  risk_name(t->risk), tier_name(t->tier),
                                  band_name(t->band), ps, (int)t->peak_session,
                                  dtype_name(t->dtype), dtier_name(t->dtype_tier),
                                  t->vendor, t->name);
                }
            }
        }

        // Periodic snapshot so a crash or a flat battery does not cost the
        // whole sweep.
        if ((uint32_t)(now - last_log) >= 30000) {
            last_log = now;
            log_snapshot(now, "PERIODIC");
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== Vulpecula ===");
    Serial.println("proximity-gated Wi-Fi + BLE recording-device sweep");

    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

    pinMode(PIN_WS2812, OUTPUT);
    led_rgb(0, 0, 30);                          // blue = booting

    spi_mux_init();
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
    pinMode(PIN_TOUCH_CS, OUTPUT); digitalWrite(PIN_TOUCH_CS, HIGH);   // once, not per read
    pinMode(PIN_SD_CS,    OUTPUT); digitalWrite(PIN_SD_CS,    HIGH);

#if !SERIAL_ONLY
    lcd_init();
    lcd_selftest();
    fill_screen(C_BLACK);
    draw_text_12(6, 6, "PROBING 5GHZ...", C_CYAN, C_BLACK);
#endif

    prox_init();
    touch_cal_load();
    if (!tracks_init()) {
        Serial.println("FATAL: track table alloc failed");
#if !SERIAL_ONLY
        fill_screen(C_RED);
        draw_text_12(6, 100, "NO PSRAM - ENABLE IT", C_WHITE, C_RED);
#endif
        while (1) { led_rgb(60,0,0); delay(400); led_rgb(0,0,0); delay(400); }
    }

    wifi_cap_init();

    Serial.println("probing 5 GHz channels...");
    wifi_cap_probe_5g();
    Serial.printf("5 GHz usable: %u channels, DFS %s\n", s_n_ch5,
                  s_dfs_ok ? "yes" : "no");
    if (s_n_ch5 == 0) {
        // Worth shouting about: 5 GHz coverage is the whole reason to build
        // this on a C5. Most other ESP32 detectors are 2.4-only and a 5 GHz
        // camera is simply invisible to them.
        Serial.println("WARNING: no 5 GHz - running 2.4 GHz only");
    }

    ble_cap_init();

    if (!log_init())
        Serial.println("SD not mounted - session will not be logged");

    memset(s_radio_ms, 0, sizeof(s_radio_ms));

#if !SERIAL_ONLY
    // First run, or a wiped NVS: calibrate touch before anything else. Guessed
    // raw limits are not worth having on a resistive panel, and an operator
    // whose taps land in the wrong place cannot even reach the SETUP screen to
    // fix it. One retry, then continue on defaults rather than trapping them
    // in a loop.
    if (!g_tcal.valid) {
        if (!touch_calibrate_interactive()) touch_calibrate_interactive();
    }
    // Radios stay off until a sweep is chosen on the welcome screen.
    sched_set_mode(MODE_IDLE);
    ui_set_screen(SCR_WELCOME);
#else
    sched_set_mode(MODE_WIFI_SWEEP);
#endif

    xTaskCreate(pump_task, "pump", 8192, NULL, 5, NULL);

    Serial.println("ready");
    Serial.println("workflow: corridor baseline -> room baseline -> Wi-Fi pass -> BLE pass");
    Serial.println("serial cmds: a=sweep-all w=wifi b=ble t=watch n=next-pass");
    Serial.println("             m=mark p=phase l=list c=rssi-cal x=stop");
    led_rgb(0, 8, 0);
}

// Serial control, so the whole tool is usable even with a dead panel.
static void serial_cmds()
{
    if (!Serial.available()) return;
    int c = Serial.read();
    uint32_t now = millis();
    if (c == 'a') { sched_set_mode(MODE_SWEEP_ALL); Serial.println("> SWEEP ALL, pass 1/2 Wi-Fi"); }
    else if (c == 'w') { sched_set_mode(MODE_WIFI_SWEEP); Serial.println("> WI-FI SWEEP"); }
    else if (c == 'b') { sched_set_mode(MODE_BLE_SWEEP); Serial.println("> BLE SWEEP"); }
    else if (c == 't') { sched_set_mode(MODE_WATCH); Serial.println("> WATCH"); }
    else if (c == 'x') { sched_set_mode(MODE_IDLE); Serial.println("> radios stopped"); }
    else if (c == 'n') { sched_all_next_pass(now); Serial.println("> pass 2/2 BLE - walk the room again"); }
    else if (c == 'm') { sched_mark_position(now); Serial.println("> position marked"); }
    else if (c == 'c') { cal_advance(); Serial.printf("> CAL: %s\n", cal_prompt()); }
    else if (c == 'p') {
        if (g_phase == PHASE_FREE)          { baseline_set_phase(PHASE_CORRIDOR, now); Serial.println("> CORRIDOR 60s"); }
        else if (g_phase == PHASE_CORRIDOR) { baseline_set_phase(PHASE_ROOM, now);     Serial.println("> ROOM 60s"); }
        else { baseline_apply_diff(); baseline_set_phase(PHASE_FREE, now);
               log_snapshot(now, "BASELINE"); Serial.println("> diff applied"); }
    } else if (c == 'l') {
        static track_t *l[MAX_TRACKS];
        uint16_t n = tracks_sorted(l, MAX_TRACKS);
        Serial.printf("--- %s | %u tracks | radio %.1fs | covers <=%.2fs | pos %lu ---\n",
                      sched_mode_name(s_mode), (unsigned)n,
                      sched_position_radio_ms()/1000.0f,
                      sched_covered_interval_ms()/1000.0f,
                      (unsigned long)s_positions);
        for (uint16_t i = 0; i < n && i < 20; i++) {
            char ps[20]; log_pseudonym(l[i]->mac, ps, sizeof(ps));
            Serial.printf("%-8s %-4s %4ddBm s%-3u %s %.20s\n",
                          tier_name(l[i]->tier_best), band_name(l[i]->band),
                          (int)l[i]->peak_session, (unsigned)l[i]->score, ps,
                          l[i]->name);
        }
    }
}

void loop()
{
#if !SERIAL_ONLY
    ui_tick(millis());
#else
    // Headless: still drive the LED so the tier is visible.
    static track_t *l[8];
    uint16_t n = tracks_sorted(l, 8);
    tier_t tr = n ? l[0]->tier : TIER_AMBIENT;
    if (tr == TIER_CONTACT)   led_rgb(50,0,0);
    else if (tr == TIER_NEAR) led_rgb(45,22,0);
    else                      led_rgb(0,8,0);
#endif
    serial_cmds();
    delay(10);
}
