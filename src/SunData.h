#ifndef SUNDATA_H
#define SUNDATA_H
#include <Arduino.h>

// Struktura danych
struct SunPosition {
  uint8_t hour;
  uint8_t minute;
  float elevation;
  float azimuth;
};

extern const SunPosition sunData[]; // Deklaracja tablicy z danymi o pozycji słońca
extern const int dataCount; // Deklaracja zmiennej z liczbą danych

#endif // SUNDATA_H