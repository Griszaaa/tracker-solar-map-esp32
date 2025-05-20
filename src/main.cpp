#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <TrackerMove.h>
#include <WiFiLogger.h>
#include <driver/timer.h>
#include "SunData.h"

// Zmienne globalne
RTC_DS3231 rtc;
TrackerMove tracker;
volatile bool timerFlag = false;
bool trackingEnabled = false;


// Konfiguracja timera
const uint64_t interval_us = 1000000; // 1 sekunda w mikrosekundach
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
    processTrackerMovement(); // ustaw się natychmiast po uruchomieniu
  }

  if (timerFlag) {
    timerFlag = false;
    if (trackingEnabled) {
      processTrackerMovement();
    }
  }
}


void initializeSystem() {
  // Serial.begin(115200); // Pozostawione do ewentualnego debugowania lokalnego
  Logger.begin("SolarTracker", "kamilcanvas");
  Wire.begin(21, 22); // SDA, SCL

  if (!rtc.begin()) {
    Logger.println("Błąd RTC!");
    while(1);
  }

  if (rtc.lostPower()) {
    Logger.println("RTC utracił zasilanie - czas może być nieprawidłowy!");
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Ustaw na czas kompilacji
  }

  tracker.begin();
  // tracker.homing(); // Homing można wykonać z poziomu klienta
}

void processTrackerMovement() {
  DateTime now = rtc.now();
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
    const SunPosition& next = sunData[i+1];
    
    if ((now.hour() > current.hour || (now.hour() == current.hour && now.minute() >= current.minute)) &&
        (now.hour() < next.hour || (now.hour() == next.hour && now.minute() < next.minute))) {
      
      if (current.elevation < 0) {
        if (tracker.getCurrentAzimuth() != 0 || tracker.getCurrentElevation() != 90) {
          tracker.homing();
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

  // Jeśli nie znaleziono pasującego wpisu w tablicy — zakończyły się dane
  tracker.homing();
  trackingEnabled = false;
  Logger.println("Brak danych o pozycji słońca — przejście w tryb czuwania (homing)");
  return false;
}


bool IRAM_ATTR onTimer(void* arg) {
  timerFlag = true;
  return true;
}
