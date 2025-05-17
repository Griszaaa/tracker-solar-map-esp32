#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <TrackerMove.h>
#include <driver/timer.h>

#include "SunData.h"

// Zmienne globalne
RTC_DS3231 rtc;
TrackerMove tracker;
volatile bool timerFlag = false;

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
  // Sprawdzenie flagi timera
  if (timerFlag) {
    timerFlag = false;
    processTrackerMovement();
  }
}
// Funkcja do inicjalizacji systemu
void initializeSystem() {
  Serial.begin(115200);

  Wire.begin(21, 22); // SDA, SCL

  if (!rtc.begin()) {
    Serial.println("Błąd RTC!");
    while(1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC utracił zasilanie - czas może być nieprawidłowy!");
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Ustaw na czas kompilacji
  }

  tracker.begin();
  tracker.homing();
}
// Funkcja do wykonywania ruchu trackerem w zależności od pozycji słońca
void processTrackerMovement() {
  DateTime now = rtc.now();
  // printCurrentTime(now);
  Serial.print("Aktualny czas: ");
  Serial.println(now.timestamp());

  float targetAzimuth, targetElevation;
  if (needsPositionChange(now, targetAzimuth, targetElevation)) {
    tracker.moveTracker(targetAzimuth, targetElevation);
  } else {
    Serial.println("Pozycja trackera nie wymaga zmiany");
  }
}
// Funkcja sprawdzająca, czy pozycja trackera wymaga zmiany
bool needsPositionChange(const DateTime& now, float& targetAzimuth, float& targetElevation) {
  for (int i = 0; i < dataCount - 1; i++) {
    const SunPosition& current = sunData[i];
    const SunPosition& next = sunData[i+1];
    
    if ((now.hour() > current.hour || (now.hour() == current.hour && now.minute() >= current.minute)) &&
        (now.hour() < next.hour || (now.hour() == next.hour && now.minute() < next.minute))) {
      
      if (current.elevation < 0) {
        if (tracker.getCurrentAzimuth() != 0 || tracker.getCurrentElevation() != 90) {
          targetAzimuth = 0;
          targetElevation = 90;
          Serial.println("Słońce poniżej horyzontu - panel w pozycji spoczynkowej");
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
  return false;
}
// Funkcja wywoływana przez timer
bool IRAM_ATTR onTimer(void* arg) {
  timerFlag = true;
  return true; // return true to yield after ISR
}

