#include <Arduino.h>
#include "SunData.h"

// Definicja struktury do przechowywania danych o pozycji słońca
const SunPosition sunData[] = {
  {4, 13, 15, 10},
  {4, 14, 5, 10},
  {4, 15, 4, 10},
  {4, 16, 3, 10},
  {4, 17, 2, 10},
  {4, 18, 1, 10},
  {4, 19, 0.5, 10},
};

// Definicja liczby danych o pozycji słońca
const int dataCount = sizeof(sunData)/sizeof(SunPosition);