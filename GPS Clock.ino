/*
  GPS-disciplined desk clock for a NodeMCU (ESP8266), with a 2.42" SSD1309
  128x64 I2C OLED display.

  Time-keeping model
  -------------------
  The displayed clock is a free-running, millis()-based clock: it ticks on
  its own every loop iteration and does NOT require a live GPS fix to keep
  working. Whenever a fresh, valid GPS timestamp arrives, that clock is
  corrected (not replaced/re-derived) to match GPS truth. This means:
    - Short GPS dropouts (indoors, obstructed sky) don't freeze or blank the
      display -- the clock just keeps ticking from its last known-good sync.
    - Long-term accuracy still comes entirely from GPS; the local clock only
      bridges the gaps between updates.

  Location, timezone name/offset, and DST come from a one-time (then every
  24h) lookup against TimeZoneDB using the GPS fix's lat/lon.

  WIRING
  ------
  GPS module (NEO-6M/ATGM336H) -- SoftwareSerial:
    GPS TX  -> NodeMCU D2 (GPIO4)   [SoftwareSerial RX side]
    GPS RX  -> NodeMCU D1 (GPIO5)   [SoftwareSerial TX side]
    VCC -> 3.3V, GND -> GND

  OLED display (HiLetgo 2.42" SSD1309 128x64, I2C) -- bit-banged SW I2C,
  deliberately NOT sharing pins with the GPS SoftwareSerial (D1/D2) or the
  hardware UART (D9/D10, used by USB/Serial Monitor):
    OLED SCL -> NodeMCU D5 (GPIO14)
    OLED SDA -> NodeMCU D6 (GPIO12)
    VCC -> 3.3V, GND -> GND

  Libraries needed (Library Manager):
    - TinyGPSPlus (by Mikal Hart)
    - ArduinoJson (by Benoit Blanchon)
    - U8g2 (by olikraus) -- for the OLED
    - ESP8266WiFi.h / ESP8266HTTPClient.h / SoftwareSerial.h are built into
      the ESP8266 Arduino core.

  Timezone API: TimeZoneDB (free key from https://timezonedb.com/register).

  If the display shows garbled/scrambled pixels instead of clean text, this
  is a very common symptom of SSD1309 clone boards needing a different
  controller init sequence -- try swapping the constructor below from
  NONAME0 to NONAME2 (both are shown, one commented out).
*/

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <U8g2lib.h>

// ---------- USER CONFIG ----------
const char* WIFI_SSID     = "Chandra";
const char* WIFI_PASSWORD = "papasmurf";

// Free key from https://timezonedb.com/register (instant, no cost).
const char* TIMEZONEDB_API_KEY = "K0P4MRQG7MB6";

#define GPS_RX_PIN 4    // NodeMCU D2 -> wired to GPS TX
#define GPS_TX_PIN 5    // NodeMCU D1 -> wired to GPS RX (optional if not configuring module)
#define GPS_BAUD   9600

#define OLED_SCL_PIN 14 // NodeMCU D5 -> wired to OLED SCL
#define OLED_SDA_PIN 12 // NodeMCU D6 -> wired to OLED SDA

#define PIR_PIN 13      // NodeMCU D7 -> wired to PIR sensor signal
const unsigned long DISPLAY_TIMEOUT_MS = 60UL * 1000UL;  // blank after 60s of no motion

// How often to re-check the timezone offset once we already have one
// (in milliseconds). DST transitions are the main reason to recheck.
const unsigned long TZ_RECHECK_INTERVAL_MS = 24UL * 60UL * 60UL * 1000UL; // 24h

// ---------- GPS STATE ----------
TinyGPSPlus gps;
SoftwareSerial GPSSerial;

bool haveOffset = false;
long utcOffsetSeconds = 0;
bool isDST = false;
String tzName = "";        // full IANA name, e.g. "America/Los_Angeles"
String tzAbbrev = "";      // short form, e.g. "PDT" -- what we actually show
double lastLat = 0.0;
double lastLon = 0.0;
bool haveLocation = false;

unsigned long lastTzCheckMs = 0;

// ---------- DISPLAY ----------
// Bit-banged ("SW") I2C directly on the pins we choose -- deliberately not
// going through the Arduino Wire library at all, which avoids a whole class
// of pin-reassignment conflicts if anything else ever touches Wire.begin().
U8G2_SSD1309_128X64_NONAME0_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL_PIN, /* data=*/ OLED_SDA_PIN, /* reset=*/ U8X8_PIN_NONE);
// If the display shows scrambled/garbled pixels instead of clean text, this
// board's clone controller likely wants the alternate init sequence --
// comment out the line above and uncomment this one instead:
// U8G2_SSD1309_128X64_NONAME2_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL_PIN, /* data=*/ OLED_SDA_PIN, /* reset=*/ U8X8_PIN_NONE);

// ---------- FREE-RUNNING LOCAL CLOCK ----------
// The clock ticks from millis() every loop, independent of GPS. It is
// corrected from GPS exactly once at startup (to acquire an initial value)
// and then once an hour thereafter (reusing the health-report interval
// below) -- NOT every second. Resyncing every second used to feed the small
// per-second variance in GPS sentence timing/processing straight into
// millisAtSync, which showed up on screen as visible jitter/skipped
// seconds. The free-running millis() clock in between corrections is far
// smoother than GPS's own per-second timing.
//   localEpochAtSync = the UTC second GPS reported at the last correction
//   millisAtSync      = the millis() value corresponding to that same second
// "Now" is always: localEpochAtSync + (millis() - millisAtSync) / 1000.
bool localClockSynced = false;
unsigned long localEpochAtSync = 0;
unsigned long millisAtSync = 0;

// ---------- PIR-DRIVEN DISPLAY POWER ----------
bool displayOn = true;
unsigned long lastMotionMs = 0;

// The main loop poll (digitalRead every pass, ~5ms cadence) is what handles
// normal motion detection and correctly keeps re-arming the timeout for as
// long as a PIR in repeatable-trigger mode holds its output HIGH -- that's
// a sustained level, not a stream of edges, so it needs level-polling, not
// just an edge interrupt, to track correctly.
//
// This interrupt exists purely as a safety net for the one case polling
// can't cover: a motion event that starts and ends entirely during one of
// the sketch's rare blocking WiFi calls (boot, and ~once/24h after), when
// loop() isn't running at all to sample the pin. The ISR does the absolute
// minimum -- set a flag -- and loop() does the real work once it resumes.
volatile bool pirInterruptFlag = false;

void IRAM_ATTR onPirRising() {
	pirInterruptFlag = true;
}

// The main loop poll (digitalRead every pass, ~5ms cadence) is what handles
// normal motion detection and correctly keeps re-arming the timeout for as
// long as a PIR in repeatable-trigger mode holds its output HIGH -- that's
// a sustained level, not a stream of edges, so it needs level-polling, not
// just an edge interrupt, to track correctly.

// Returns the current UTC epoch second from the free-running local clock.
// Only meaningful once localClockSynced is true.
unsigned long getCurrentUtcEpoch() {
  return localEpochAtSync + (millis() - millisAtSync) / 1000UL;
}

// ---------- NMEA SENTENCE FRAMING ----------
// Per the NMEA 0183 spec: every sentence starts with '$' (or '!' for some
// AIS/proprietary sentences) and ends with <CR><LF>; max length is 82 bytes
// including the delimiters. We frame sentences ourselves: buffer until we
// see the '$'/'!' that starts one, keep buffering until the terminating
// '\n', then hand the whole clean sentence to gps.encode() in one go.
// Stray/garbled bytes between sentences are simply discarded -- the next
// '$' always resyncs us.
#define NMEA_MAX_LEN 90   // 82 per spec + margin for the odd longer sentence
char nmeaBuf[NMEA_MAX_LEN];
uint8_t nmeaLen = 0;
bool nmeaInProgress = false;

void handleGpsByte(char c) {
  if (c == '$' || c == '!') {
    nmeaLen = 0;
    nmeaBuf[nmeaLen++] = c;
    nmeaInProgress = true;
    return;
  }

  if (!nmeaInProgress) {
    return; // byte outside any sentence — noise, drop it
  }

  if (nmeaLen >= NMEA_MAX_LEN - 1) {
    nmeaInProgress = false;
    nmeaLen = 0;
    return;
  }

  nmeaBuf[nmeaLen++] = c;

  if (c == '\n') {
    nmeaBuf[nmeaLen] = '\0';
    //Serial.print(nmeaBuf);

    for (uint8_t i = 0; i < nmeaLen; i++) {
      gps.encode(nmeaBuf[i]);
    }

    nmeaInProgress = false;
    nmeaLen = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Keep the WiFi radio fully off until we actually need it. Left on, its
  // interrupt activity corrupts SoftwareSerial's bit-banged timing badly
  // enough to break NMEA checksums, which is why fixes silently fail even
  // when the GPS itself is working (garbled bytes make gps.encode() reject
  // otherwise-valid sentences).
  WiFi.mode(WIFI_OFF);

  // Pin/config/buffer all set here rather than in the constructor for this
  // version of EspSoftwareSerial. 1024-byte buffer gives plenty of headroom
  // over the small default.
  GPSSerial.begin(GPS_BAUD, SWSERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN, false, 1024, 0);

  // Restrict the module to GPS only (it defaults to GPS+BDS combined, which
  // roughly doubles the GSV satellite-list sentence volume). Verified command
  // + checksum from the ATGM336H documentation. Takes effect immediately;
  // add a $PCAS00*01 save-to-flash command afterward if you want it to
  // persist across power cycles instead of resetting to GPS+BDS each boot.
  GPSSerial.print("$PCAS04,1*18\r\n");

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 20, "GPS Clock");
  u8g2.drawStr(0, 34, "Waiting for");
  u8g2.drawStr(0, 46, "GPS fix...");
  u8g2.sendBuffer();

  // Most HC-SR501-style PIR modules idle low and pulse high on motion; no
  // internal pull needed since the module actively drives the pin.
  pinMode(PIR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), onPirRising, RISING);
  lastMotionMs = millis(); // start "on" rather than immediately timing out

  Serial.println("Waiting for GPS fix...");
}

// Corrects the free-running local clock to match GPS. gps.time.age() is how
// many ms have elapsed since that HH:MM:SS was actually valid (decode/
// processing delay) -- subtracting it aligns millisAtSync to the real
// moment that second began, rather than to whenever we happened to get
// around to processing the sentence.
void resyncLocalClockFromGps() {
  long utcDays = daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day());
  unsigned long utcEpoch = (unsigned long)utcDays * 86400UL
                          + (unsigned long)gps.time.hour() * 3600UL
                          + (unsigned long)gps.time.minute() * 60UL
                          + (unsigned long)gps.time.second();

  localEpochAtSync = utcEpoch;
  millisAtSync = millis() - gps.time.age();
  localClockSynced = true;
}

// Fixed-grid tick scheduler for the display refresh. Deliberately NOT a
// blocking delay(1000) at the end of loop(): OLED redraw over bit-banged SW
// I2C can itself take a non-trivial (and variable) number of milliseconds
// to push a full 1024-byte framebuffer, so "draw, then delay(1000)" gives a
// real loop period of (draw time) + 1000ms -- consistently over a second.
// Since the epoch is only sampled once per loop pass, any pass whose total
// period creeps past 2000ms causes a whole displayed second to be skipped,
// even though the underlying millis()-based clock itself never drifts.
// Scheduling the next tick as "previous tick + 1000ms" (not "now + 1000ms")
// keeps this locked to a fixed 1Hz grid with no long-term drift, and
// polling for it with a short delay (instead of blocking for a full
// second) means we always catch the boundary within a few ms.
unsigned long nextTickMs = 0;

void loop() {
  // Feed the parser one fully-framed sentence at a time.
  while (GPSSerial.available() > 0) {
    handleGpsByte((char)GPSSerial.read());
  }

  // Acquire the local clock's initial value from GPS exactly once, as soon
  // as a valid timestamp is available. After this, correction only happens
  // on the hourly cadence below, not every second -- see the comment on
  // the local-clock globals above for why.
  if (!localClockSynced && gps.time.isValid() && gps.date.isValid()) {
    resyncLocalClockFromGps();
  }

  if (gps.location.isValid() && gps.location.isUpdated()) {
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
    haveLocation = true;

    bool needCheck = !haveOffset ||
                     (millis() - lastTzCheckMs > TZ_RECHECK_INTERVAL_MS);

    if (needCheck) {
      Serial.printf("Fix: lat=%.6f lon=%.6f — querying timezone...\n", lastLat, lastLon);
      if (fetchUtcOffset(lastLat, lastLon)) {
        lastTzCheckMs = millis();
        haveOffset = true;
        Serial.printf("Timezone: %s (%s)  UTC offset: %+.2f h  DST: %s\n",
                      tzName.c_str(), tzAbbrev.c_str(), utcOffsetSeconds / 3600.0,
                      isDST ? "yes" : "no");
      } else {
        Serial.println("Timezone lookup failed, will retry next fix.");
      }
    }
  }

  // Periodically report parse health to Serial, and -- on this same hourly
  // cadence -- correct the free-running local clock against GPS. This is
  // the only resync after the initial one above; see the comment on the
  // local-clock globals for why it's not done every second.
  static unsigned long lastHealthMs = 0;
  if (millis() - lastHealthMs > 60000UL * 60UL) {
    lastHealthMs = millis();
    unsigned long passed = gps.passedChecksum();
    unsigned long failed = gps.failedChecksum();
    double healthPct = (passed > 0) ? (1.0 - (double)failed / (double)passed) * 100.0 : 0.0;
    Serial.printf("GPSSerial health=%.1f%% sats=%d chars processed=%lu\n",
                  healthPct, gps.satellites.value(), gps.charsProcessed());

    if (gps.time.isValid() && gps.date.isValid()) {
      resyncLocalClockFromGps();
    }
  }

  // Combine the direct level read (handles sustained presence correctly —
  // a PIR in repeatable-trigger mode holds HIGH continuously, not as a
  // stream of edges) with the interrupt flag (catches a motion blip that
  // started and ended entirely during a blocking WiFi call, which the
  // level read alone could otherwise miss since loop() wasn't running to
  // sample it).
  bool motionNow = (digitalRead(PIR_PIN) == HIGH) || pirInterruptFlag;
  pirInterruptFlag = false; // consumed either way

  if (motionNow) {
	  lastMotionMs = millis();
	  if (!displayOn) {
		  u8g2.sleepOff();
		  displayOn = true;
	  }
  }
  else if (displayOn && (millis() - lastMotionMs > DISPLAY_TIMEOUT_MS)) {
	  u8g2.sleepOn();
	  displayOn = false;
  }

  // Fire at (or very shortly after) each fixed 1000ms boundary, rather than
  // sleeping for a full second regardless of how long the rest of the loop
  // body took. (long) cast handles millis() rollover correctly.
  if ((long)(millis() - nextTickMs) >= 0) {
	  nextTickMs += 1000;
	  updateDisplay();
  }

  delay(5); // keep the loop responsive to the tick boundary without
            // busy-spinning; also yields time for the SDK's background
            // housekeeping (WiFi/RTOS), same as a delay(0)/yield() would.
}

// ---------- DISPLAY RENDERING ----------
void updateDisplay() {
  u8g2.clearBuffer();

  if (!localClockSynced) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 20, "GPS Clock");
    u8g2.drawStr(0, 34, "Waiting for");
    u8g2.drawStr(0, 46, "GPS fix...");
    static const char spinner[] = {'|', '/', '-', '\\'};
    char spin[2] = { spinner[(millis() / 500) % 4], '\0' };
    u8g2.drawStr(112, 62, spin);
    u8g2.sendBuffer();
    return;
  }

  // Recompute passed/failed checksum health + sat count every frame so the
  // on-screen "signal health" is always current, independent of the
  // once-an-hour Serial report above.
  unsigned long passed = gps.passedChecksum();
  unsigned long failed = gps.failedChecksum();
  double healthPct = (passed > 0) ? (1.0 - (double)failed / (double)passed) * 100.0 : 0.0;
  int sats = gps.satellites.value();

  unsigned long utcEpoch = getCurrentUtcEpoch();
  long offsetSeconds = haveOffset ? utcOffsetSeconds : 0;
  long localEpoch = (long)utcEpoch + offsetSeconds;

  long localDays = (localEpoch >= 0) ? localEpoch / 86400L
                                      : (localEpoch - 86399L) / 86400L;
  long secOfDay = localEpoch - localDays * 86400L;

  int y, mo, d;
  civilFromDays(localDays, y, mo, d);
  int hh = secOfDay / 3600;
  int mm = (secOfDay % 3600) / 60;
  int ss = secOfDay % 60;

  static const char* monthNames[] = {
    "Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"
  };
  static const char* dayNames[] = {
    "Sun","Mon","Tue","Wed","Thu","Fri","Sat"
  };
  // Epoch day 0 (1970-01-01) was a Thursday.
  int weekday = (int)(((localDays % 7) + 7 + 4) % 7);

  // --- Big HH:MM:SS
  char bigTime[9];
  snprintf(bigTime, sizeof(bigTime), "%02d:%02d:%02d", hh, mm, ss);
  u8g2.setFont(u8g2_font_logisoso28_tn);
  int bigW = u8g2.getUTF8Width(bigTime);
  u8g2.drawStr((128 - bigW) / 2, 32, bigTime);

  // --- Date line ---
  char dateLine[24];
  snprintf(dateLine, sizeof(dateLine), "%s, %s %d %04d", dayNames[weekday], monthNames[mo - 1], d, y);
  u8g2.setFont(u8g2_font_6x10_tf);
  int dateW = u8g2.getUTF8Width(dateLine);
  u8g2.drawStr((128 - dateW) / 2, 42, dateLine);

  // --- Location line ---
  char locLine[24];
  if (haveLocation) {
    snprintf(locLine, sizeof(locLine), "%.4f%c %.4f%c",
             fabs(lastLat), lastLat >= 0 ? 'N' : 'S',
             fabs(lastLon), lastLon >= 0 ? 'E' : 'W');
  } else {
    snprintf(locLine, sizeof(locLine), "location: --");
  }
  int locW = u8g2.getUTF8Width(locLine);
  u8g2.drawStr((128 - locW) / 2, 52, locLine);

  // --- Timezone + signal health line ---
  char statusLine[28];
  if (haveOffset) {
    snprintf(statusLine, sizeof(statusLine), "%s UTC%+ld Sats:%d %.0f%%",
             tzAbbrev.length() ? tzAbbrev.c_str() : tzName.c_str(),
             offsetSeconds / 3600, sats, healthPct);
  } else {
    snprintf(statusLine, sizeof(statusLine), "TZ: -- Sats:%d %.0f%%", sats, healthPct);
  }
  u8g2.setFont(u8g2_font_5x8_tf);
  int statusW = u8g2.getUTF8Width(statusLine);
  u8g2.drawStr((128 - statusW) / 2, 62, statusLine);

  u8g2.sendBuffer();
}

// Connects to WiFi (if not already), queries TimeZoneDB for the offset at
// this lat/lon, and updates utcOffsetSeconds / isDST / tzName / tzAbbrev.
// Returns true on success.
bool fetchUtcOffset(double lat, double lon) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to WiFi");
    WiFi.mode(WIFI_STA);   // radio turns on here — first time it's touched
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
      delay(300);
      Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connect failed.");
      return false;
    }
  }

  // TimeZoneDB serves plain HTTP, so no TLS/WiFiClientSecure needed here.
  WiFiClient client;

  HTTPClient http;
  String url = buildTimezoneUrl(lat, lon);
  http.begin(client, url);
  int httpCode = http.GET();

  bool ok = false;
  if (httpCode == 200) {
    String payload = http.getString();
    ok = parseTimezoneResponse(payload);
  } else {
    Serial.printf("HTTP GET failed, code: %d\n", httpCode);
  }

  http.end();
  WiFi.mode(WIFI_OFF);
  return ok;
}

// TimeZoneDB: GET http://api.timezonedb.com/v2.1/get-time-zone?key=..&format=json&by=position&lat=..&lng=..
String buildTimezoneUrl(double lat, double lon) {
  String url = "http://api.timezonedb.com/v2.1/get-time-zone?key=";
  url += TIMEZONEDB_API_KEY;
  url += "&format=json&by=position&lat=";
  url += String(lat, 6);
  url += "&lng=";
  url += String(lon, 6);
  return url;
}

// Example TimeZoneDB response:
// {"status":"OK","message":"","countryCode":"US","countryName":"United States",
//  "zoneName":"America\/Los_Angeles","abbreviation":"PDT","gmtOffset":-25200,
//  "dst":"1", ... }
// gmtOffset is already in seconds — no unit conversion needed.
bool parseTimezoneResponse(const String& payload) {
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("JSON parse failed: ");
    Serial.println(err.c_str());
    return false;
  }

  const char* status = doc["status"] | "";
  if (strcmp(status, "OK") != 0) {
    const char* msg = doc["message"] | "(no message)";
    Serial.printf("TimeZoneDB error status: %s (%s)\n", status, msg);
    return false;
  }

  if (!doc.containsKey("zoneName") || !doc.containsKey("gmtOffset")) {
    Serial.println("Unexpected response shape.");
    return false;
  }

  tzName = doc["zoneName"].as<String>();
  utcOffsetSeconds = doc["gmtOffset"].as<long>();
  tzAbbrev = doc["abbreviation"] | "";

  const char* dstStr = doc["dst"] | "0";
  isDST = (strcmp(dstStr, "1") == 0);

  return true;
}

// Days since 1970-01-01 for a proleptic-Gregorian civil date, and its
// inverse. This is the standard Howard Hinnant algorithm — correct for any
// date, and (unlike relying on the platform's mktime()/timezone setup)
// doesn't depend on any environment configuration, so it handles
// month/year rollover exactly the same way on any platform.
static long daysFromCivil(int y, int m, int d) {
  y -= (m <= 2);
  long era = (y >= 0 ? y : y - 399) / 400;
  int yoe = (int)(y - era * 400);
  int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097L + (long)doe - 719468L;
}

static void civilFromDays(long z, int &y, int &m, int &d) {
  z += 719468L;
  long era = (z >= 0 ? z : z - 146096L) / 146097L;
  int doe = (int)(z - era * 146097L);
  int yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  y = yoe + (int)(era * 400L);
  int doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  int mp = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  m = mp + (mp < 10 ? 3 : -9);
  y += (m <= 2);
}
