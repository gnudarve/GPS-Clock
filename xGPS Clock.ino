/*
  Combined GPS for a NodeMCU (ESP8266).

    - GPS (NEO-6M/ATGM336H via plain SoftwareSerial, polled in loop()): gives
      UTC time/date directly, plus lat/lon which is used ONCE (and then every
      24h) to look up the local UTC offset/DST from TimeZoneDB over WiFi.
  WIRING
  ------
  GPS module:
    NEO-6M/ATGM336H TX  -> NodeMCU D2 (GPIO4)   [SoftwareSerial RX side]
    NEO-6M/ATGM336H RX  -> NodeMCU D1 (GPIO5)   [SoftwareSerial TX side]
    VCC -> 3.3V, GND -> GND

  Libraries needed (Library Manager):
    - TinyGPSPlus (by Mikal Hart)
    - ArduinoJson (by Benoit Blanchon)
    - ESP8266WiFi.h / ESP8266HTTPClient.h / SoftwareSerial.h are built into
      the ESP8266 Arduino core.

  Timezone API: TimeZoneDB (free key from https://timezonedb.com/register).
*/

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// ---------- USER CONFIG ----------
const char* WIFI_SSID     = "Chandra";
const char* WIFI_PASSWORD = "papasmurf";

// Free key from https://timezonedb.com/register (instant, no cost).
const char* TIMEZONEDB_API_KEY = "K0P4MRQG7MB6";

#define GPS_RX_PIN 4    // NodeMCU D2 -> wired to GPS TX
#define GPS_TX_PIN 5    // NodeMCU D1 -> wired to GPS RX (optional if not configuring module)
#define GPS_BAUD   9600

// How often to re-check the timezone offset once we already have one
// (in milliseconds). DST transitions are the main reason to recheck.
const unsigned long TZ_RECHECK_INTERVAL_MS = 24UL * 60UL * 60UL * 1000UL; // 24h

// ---------- GPS STATE ----------
TinyGPSPlus gps;
SoftwareSerial GPSSerial;

bool haveOffset = false;
long utcOffsetSeconds = 0;
bool isDST = false;
String tzName = "";

unsigned long lastTzCheckMs = 0;

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
    // Start of a new sentence — even if we were mid-sentence, this always
    // wins: either the previous one just finished normally, or it was
    // truncated/corrupted and we're resyncing on the next valid start.
    nmeaLen = 0;
    nmeaBuf[nmeaLen++] = c;
    nmeaInProgress = true;
    return;
  }

  if (!nmeaInProgress) {
    return; // byte outside any sentence — noise, drop it
  }

  if (nmeaLen >= NMEA_MAX_LEN - 1) {
    // Overran the max sentence length without seeing a terminator — this
    // sentence is malformed, abandon it and wait for the next '$'.
    nmeaInProgress = false;
    nmeaLen = 0;
    return;
  }

  nmeaBuf[nmeaLen++] = c;

  if (c == '\n') {
    // Complete sentence (ends '\r\n', we just buffered the '\n').
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
  // over the small default — should comfortably absorb a burst of
  // GGA+GLL+GSA+GSV(x3)+RMC+VTG+ZDA even if the main loop is briefly delayed
  // servicing it.
  GPSSerial.begin(GPS_BAUD, SWSERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN, false, 1024, 0);

  // Restrict the module to GPS only (it defaults to GPS+BDS combined, which
  // roughly doubles the GSV satellite-list sentence volume). Verified command
  // + checksum from the ATGM336H documentation. Takes effect immediately;
  // add a $PCAS00*01 save-to-flash command afterward if you want it to
  // persist across power cycles instead of resetting to GPS+BDS each boot.
  GPSSerial.print("$PCAS04,1*18\r\n");

  Serial.println("Waiting for GPS fix...");
}

void loop() {
  // Feed the parser one fully-framed sentence at a time.
  while (GPSSerial.available() > 0) {
    handleGpsByte((char)GPSSerial.read());
  }

  if (gps.location.isValid() && gps.location.isUpdated()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();

    bool needCheck = !haveOffset ||
                     (millis() - lastTzCheckMs > TZ_RECHECK_INTERVAL_MS);

    if (needCheck) {
      Serial.printf("Fix: lat=%.6f lon=%.6f — querying timezone...\n", lat, lon);
      if (fetchUtcOffset(lat, lon)) {
        lastTzCheckMs = millis();
        haveOffset = true;
        Serial.printf("Timezone: %s  UTC offset: %+.2f h  DST: %s\n",
                      tzName.c_str(), utcOffsetSeconds / 3600.0,
                      isDST ? "yes" : "no");

		if (haveOffset && gps.time.isValid() && gps.date.isValid()) {
			// GPS time/date fields are always UTC.
			printLocalTime(utcOffsetSeconds);
		}
	  } else {
        Serial.println("Timezone lookup failed, will retry next fix.");
      }
    }
  }

  // Periodically report parse health — if the pass rate keeps dropping,
  // bytes are still being lost/corrupted upstream of this framing
  // (SoftwareSerial timing, wiring, baud mismatch).
  static unsigned long lastHealthMs = 0;
  if (millis() - lastHealthMs > 60000 * 60) {
    lastHealthMs = millis();
    unsigned long passed = gps.passedChecksum();
    unsigned long failed = gps.failedChecksum();
    double healthPct = (passed > 0) ? (1.0 - (double)failed / (double)passed) * 100.0 : 0.0;
    Serial.printf("GPSSerial health=%.1f%% sats=%d chars processed=%d\n", healthPct, gps.satellites.value(), gps.charsProcessed());

	if (haveOffset && gps.time.isValid() && gps.date.isValid()) {
		// GPS time/date fields are always UTC.
		printLocalTime(utcOffsetSeconds);
	}
  }

  delay(1000);
}

// Connects to WiFi (if not already), queries TimeZoneDB for the offset at
// this lat/lon, and updates utcOffsetSeconds / isDST / tzName.
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

  // TimeZoneDB serves plain HTTP, so no TLS/WiFiClientSecure needed here —
  // simpler and lighter on the ESP8266 than the HTTPS round we did earlier.
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

void printLocalTime(long offsetSeconds) {
  // GPS date/time fields are always UTC. Convert to a Unix-style epoch
  // (seconds since 1970-01-01 UTC), apply the offset, then convert back —
  // this correctly rolls over day, month, and year boundaries together,
  // rather than adjusting the day-of-month in isolation.
  long utcDays = daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day());
  long utcEpoch = utcDays * 86400L
                 + gps.time.hour() * 3600L
                 + gps.time.minute() * 60L
                 + gps.time.second();

  long localEpoch = utcEpoch + offsetSeconds;

  // Floor division so this stays correct even if offsetSeconds ever pushed
  // localEpoch negative (not a real concern at today's epoch values, but
  // cheap to get right).
  long localDays = (localEpoch >= 0) ? localEpoch / 86400L
                                      : (localEpoch - 86399L) / 86400L;
  long secOfDay = localEpoch - localDays * 86400L;

  int outYear, outMonth, outDay;
  civilFromDays(localDays, outYear, outMonth, outDay);

  int outHour = secOfDay / 3600;
  int outMin  = (secOfDay % 3600) / 60;
  int outSec  = secOfDay % 60;

  Serial.printf("Local time (%s): %04d-%02d-%02d  %02d:%02d:%02d\n",
                tzName.c_str(), outYear, outMonth, outDay,
                outHour, outMin, outSec);
}
