#include <Wire.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <MS5837.h>
#include <SPIFFS.h>

MS5837 ms;
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

unsigned long lastPrint = 0;

bool loggingStarted = true;      // Start logging on startup
bool hasSurfaceFix  = false;
bool msOK           = false;

double surfaceLat = 0.0;
double surfaceLng = 0.0;

File logFile;

// Seattle time zone offset: UTC-8 in winter (PST), UTC-7 in summer (PDT)
const int TIMEZONE_OFFSET_HOURS = -8;

// Get UTC timestamp string
String getUtcTimestamp() {
  if (gps.date.isValid() && gps.time.isValid()) {
    char buf[25];
    snprintf(buf, sizeof(buf),
             "%04d-%02d-%02d %02d:%02d:%02d",
             gps.date.year(),
             gps.date.month(),
             gps.date.day(),
             gps.time.hour(),
             gps.time.minute(),
             gps.time.second());
    return String(buf);
  }
  return String("0000-00-00 00:00:00");
}

// Seattle time
String getLocalTimestamp() {

  // GPS time not ready yet
  if (!gps.date.isValid() || !gps.time.isValid()
      || gps.date.year() < 2024) {
    return String("WAITING_FOR_GPS_TIME");
  }

  int year   = gps.date.year();
  int month  = gps.date.month();
  int day    = gps.date.day();
  int hour   = gps.time.hour() - 8;   // Seattle time（PST）
  int minute = gps.time.minute();
  int second = gps.time.second();

  if (hour < 0) {
    hour += 24;
    day -= 1;
  }

  char buf[25];
  snprintf(buf, sizeof(buf),
           "%04d-%02d-%02d %02d:%02d:%02d",
           year, month, day, hour, minute, second);

  return String(buf);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== MS5837 + GPS logger (Seattle local time) ===");

  // I2C
  Wire.begin(21, 22);

  // MS5837 init
  Serial.println("[MS5837] init...");
  if (!ms.init()) {
    Serial.println("[MS5837] init FAILED. Please check wiring / library。");
    msOK = false;
  } else {
    ms.setModel(MS5837::MS5837_02BA); // Use 30BA if applicable
    ms.setFluidDensity(997);          // fresh water
    Serial.println("[MS5837] init OK.");
    msOK = true;
  }

  // GPS UART2
  GPSSerial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("[GPS] UART started on RX16/TX17 @9600");

  // SPIFFS
  Serial.print("[SPIFFS] Mounting...");
  if (!SPIFFS.begin(true)) {
    Serial.println(" FAILED!");
  } else {
    Serial.println(" done.");
    logFile = SPIFFS.open("/log.csv", FILE_APPEND);
    if (!logFile) {
      Serial.println("[SPIFFS] Could not open /log.csv");
    } else {
      if (logFile.size() == 0) {
        // Use timestamp_local_seattle as the first column, and also store UTC time for easier time reference
        logFile.println("timestamp_local_seattle,timestamp_utc,pressure_mbar,temp_C,depth_m,gps_lat,gps_lng,satellites,hdop");
        logFile.flush();
      }
      Serial.println("[SPIFFS] /log.csv ready.");
    }
  }

  loggingStarted = true;
  Serial.println("[LOG] loggingStarted = true");
}

void loop() {
  // Read GPS 
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    gps.encode(c);
  }

  // Update surface fix
  if (gps.location.isValid()) {
    surfaceLat   = gps.location.lat();
    surfaceLng   = gps.location.lng();
    hasSurfaceFix = true;
  }

  // Read MS5837
  if (msOK) {
    ms.read();
  }

  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;

    String localTS = getLocalTimestamp();
    String utcTS   = getUtcTimestamp();

    float pressure = msOK ? ms.pressure()    : NAN;
    float temp     = msOK ? ms.temperature() : NAN;
    float depth    = msOK ? ms.depth()       : NAN;

    // Serial Monitor Output
    Serial.println("===== SAMPLE =====");
    Serial.print("Local time (Seattle): ");
    Serial.println(localTS);
    Serial.print("UTC from GPS:         ");
    Serial.println(utcTS);

    if (msOK) {
      Serial.print("Depth: ");
      Serial.print(depth, 3);
      Serial.print(" m | Temp: ");
      Serial.print(temp, 2);
      Serial.print(" °C | Pressure: ");
      Serial.print(pressure, 2);
      Serial.println(" mbar");
    } else {
      Serial.println("MS5837: init FAILED → no pressure/depth data");
    }

    Serial.print("GPS charsProcessed: ");
    Serial.println(gps.charsProcessed());

    Serial.print("GPS fix: ");
    Serial.println(gps.location.isValid() ? "YES" : "NO");

    if (hasSurfaceFix) {
      Serial.print("Lat: ");
      Serial.print(surfaceLat, 6);
      Serial.print(" | Lng: ");
      Serial.println(surfaceLng, 6);
    } else {
      Serial.println("Lat/Lng: NO FIX (usually unavailable indoors, works outdoors)");
    }

    Serial.print("Satellites: ");
    Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
    Serial.print(" | HDOP: ");
    Serial.println(gps.hdop.isValid() ? gps.hdop.hdop() : 99.99);

    Serial.println("[LOG] 1 line appended to /log.csv");
    Serial.println("====================\n");

    // 寫入 SPIFFS CSV
    if (loggingStarted && logFile) {
      logFile.print(localTS); logFile.print(',');
      logFile.print(utcTS);   logFile.print(',');

      if (msOK) {
        logFile.print(pressure, 2); logFile.print(',');
        logFile.print(temp, 2);     logFile.print(',');
        logFile.print(depth, 3);    logFile.print(',');
      } else {
        logFile.print("NaN,NaN,NaN,");
      }

      if (hasSurfaceFix) {
        logFile.print(surfaceLat, 6); logFile.print(',');
        logFile.print(surfaceLng, 6); logFile.print(',');
      } else {
        logFile.print("NaN,NaN,");
      }

      if (gps.satellites.isValid()) {
        logFile.print(gps.satellites.value());
      } else {
        logFile.print(0);
      }
      logFile.print(',');

      if (gps.hdop.isValid()) {
        logFile.print(gps.hdop.hdop());
      } else {
        logFile.print(99.99);
      }
      logFile.println();

      logFile.flush();
    }
  }
}
