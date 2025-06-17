#include <Arduino.h>
#include <WiFi.h>
#include <TrackerMove.h>
#include <WiFiLogger.h>
#include <driver/timer.h>
#include <RTClib.h>  // Zachowujemy DateTime do wygodnych operacji czasowych
#include "SunData.h"

#define WIFI_LED 3 // Pin diody LED Wi-Fi


// Zmienne globalne
TrackerMove tracker;
volatile bool timerFlag = false;
bool trackingEnabled = false;
String startupLog; // Bufor na logi startowe

// Ustawienia Wi-Fi i NTP
// const char* ssid = "WiadomoCoTegoTen";
// const char* password = "J3bacA1R20";
const char* ssid = "Redmi Note 12 Pro";
const char* password = "12345678";
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;         // GMT+1
const int daylightOffset_sec = 3600;     // +1h dla czasu letniego

// Konfiguracja timera
const uint64_t interval_us = 10000000; // 10 sekund w mikrosekundach
timer_config_t timerConfig = {
    .alarm_en = TIMER_ALARM_EN,
    .counter_en = TIMER_PAUSE,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .divider = 80 // 80 MHz / 80 = 1 MHz (1 tick = 1 us)
};

// Deklaracje funkcji
void initializeSystem();
void syncTimeWithNTP();
void processTrackerMovement();
bool needsPositionChange(const DateTime& now, float& targetAzimuth, float& targetElevation);
bool IRAM_ATTR onTimer(void* arg);

void setup() {

  // Inicjalizacja timera
  timer_init(TIMER_GROUP_0, TIMER_0, &timerConfig);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, interval_us);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, &onTimer, NULL, 0);
  timer_start(TIMER_GROUP_0, TIMER_0);

  initializeSystem();
}

void loop() {
  Logger.handleClient();

  String cmd = Logger.readCommand();
  if (cmd == "homing") {
    Logger.println("Wykonywanie homingu...");
    trackingEnabled = false;
    tracker.homing();
  } else if (cmd == "start") {
    Logger.println("Tryb śledzenia aktywowany.");
    trackingEnabled = true;
    processTrackerMovement();
  } else if (cmd == "stop") {
    Logger.println("Tryb śledzenia dezaktywowany.");
    trackingEnabled = false;
  } else if (cmd == "syncTime") {
    trackingEnabled = false;
    Logger.println("Synchronizacja czasu z NTP...");
    Logger.println("Logi przez Serial...");
    syncTimeWithNTP();
  } else if (cmd == "move") {
    if (trackingEnabled) {
      Logger.println("Dezaktywuj tryb śledzenia przed ręcznym ustawieniem pozycji trackera.");
      return;
    }
    Logger.println("Ręczne ustawienie pozycji trackera.\nPodaj azymut:");
    float azimuth = 0.0f;
    while (true) {
      String azStr = Logger.readCommand();
      if (azStr.length() > 0) {
        azimuth = azStr.toFloat();
        break;
      }
      delay(10);
    }

    Logger.println("Podaj elewację:");
    float elevation = 0.0f;
    while (true) {
      String elStr = Logger.readCommand();
      if (elStr.length() > 0) {
        elevation = elStr.toFloat();
        break;
      }
      delay(10);
    }

    tracker.moveTracker(azimuth, elevation);
  } else if (cmd == "status") {
    if (!trackingEnabled) {
      Logger.println("Tryb śledzenia jest wyłączony.");
    } else {
      Logger.println("Tryb śledzenia jest aktywny.");
    }
    Logger.print("Aktualna pozycja trackera: Azymut = ");
    Logger.print(String(tracker.getCurrentAzimuth()));
    Logger.print(", Elewacja = ");
    Logger.println(String(tracker.getCurrentElevation()));
  } 

  if (timerFlag) {
    timerFlag = false;
    if (trackingEnabled) {
      processTrackerMovement();
    }
  }
  digitalWrite(WIFI_LED, Logger.isClientConnected() ? LOW : HIGH); // Włącz diodę LED, gdy tryb śledzenia jest aktywny
}

void initializeSystem() {
  WiFi.mode(WIFI_AP);
  Logger.begin("SolarTracker", "SolarTracker", 23);
  tracker.begin();
  
  pinMode(WIFI_LED, OUTPUT);
  digitalWrite(WIFI_LED, HIGH); // Wyłącz diodę LED
}

void syncTimeWithNTP() {
  Serial.begin(115200);
  Serial.println("Rozpoczynanie synchronizacji czasu NTP...");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setHostname("SolarTrackerESP32");


  Serial.println("Łączenie z WiFi: ");

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 30) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Nie udało się połączyć z WiFi — pomijam synchronizację.");
    return;
  }

  Serial.println("Połączono z WiFi!");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Czekam na synchronizację czasu NTP...");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Błąd pobierania czasu przez NTP");
    delay(1000);
  }

  char timeBuffer[32];
  strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  Serial.print("Czas NTP: ");
  Serial.println(timeBuffer);
  WiFi.disconnect(true);
  Serial.println("Powrót do trybu Access Point — Logger gotowy!");
  WiFi.mode(WIFI_AP);
  Logger.begin("SolarTracker", "SolarTracker", 23);
}

void processTrackerMovement() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Logger.println("Błąd pobierania lokalnego czasu!");
    return;
  }
  DateTime now(
    timeinfo.tm_year + 1900,
    timeinfo.tm_mon + 1,
    timeinfo.tm_mday,
    timeinfo.tm_hour,
    timeinfo.tm_min,
    timeinfo.tm_sec
  );

  Logger.print("Aktualny czas: ");
  Logger.println(now.timestamp());

  float targetAzimuth, targetElevation;
  if (needsPositionChange(now, targetAzimuth, targetElevation)) {
    tracker.moveTracker(targetAzimuth, targetElevation);
  } else {
    Logger.println("Pozycja trackera nie wymaga zmiany");
  }
}

bool needsPositionChange(const DateTime& now, float& targetAzimuth, float& targetElevation) {
    for (int i = 0; i < dataCount - 1; i++) {
        const SunPosition& current = sunData[i];
        const SunPosition& next = sunData[i + 1];

        DateTime currentTime(now.year(), now.month(), now.day(), current.hour, current.minute, 0);
        DateTime nextTime(now.year(), now.month(), now.day(), next.hour, next.minute, 0);

        if (now >= currentTime && now < nextTime) {
            if (current.elevation < 0) {
                if (tracker.getCurrentAzimuth() != 0 || tracker.getCurrentElevation() != 90) {
                    targetAzimuth = 0.0;
                    targetElevation = 90.0;
                    Logger.println("Słońce poniżej horyzontu - panel w pozycji spoczynkowej");
                    return true;
                }
                return false;
            }

            targetAzimuth = current.azimuth;
            targetElevation = 90.0 - current.elevation;
            return (abs(tracker.getCurrentAzimuth() - targetAzimuth) > 0.1 ||
                    abs(tracker.getCurrentElevation() - targetElevation) > 0.1);
        }
    }

    const SunPosition& last = sunData[dataCount - 1];
    DateTime lastTime(now.year(), now.month(), now.day(), last.hour, last.minute, 0);
    DateTime graceUntil = lastTime + TimeSpan(0, 0, 15, 0);
    Logger.println("Koniec zakresu danych - ustawiona ostatnia znana pozycja Słońca.");

    if (now >= lastTime && now < graceUntil) {
        targetAzimuth = last.azimuth;
        targetElevation = 90.0 - last.elevation;
        return (abs(tracker.getCurrentAzimuth() - targetAzimuth) > 0.1 ||
                abs(tracker.getCurrentElevation() - targetElevation) > 0.1);
    }

    Logger.println("Brak danych o pozycji słońca — przejście w tryb czuwania (homing)");
    tracker.homing();
    trackingEnabled = false;
    return false;
}

bool IRAM_ATTR onTimer(void* arg) {
  timerFlag = true;
  return true;
}
