#include "TrackerMove.h"
#include <Arduino.h>
#include <math.h>

// Konstruktor klasy TrackerMove
// Inicjalizuje silniki i zmienne za pomocą listy inicjalizacyjnej
TrackerMove::TrackerMove()
    : motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN),
      motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN),
      currentAzimuth(0.0), currentElevation(90.0), minElevation(0.0) {}
// Funkcja inicjalizująca
// Ustawia piny, prędkości i akceleracje silników
void TrackerMove::begin() {
    pinMode(MOTOR1_EN_PIN, OUTPUT);
    pinMode(MOTOR2_EN_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_1, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);

    digitalWrite(MOTOR1_EN_PIN, HIGH);
    digitalWrite(MOTOR2_EN_PIN, HIGH);

    motor1.setMaxSpeed(SPEED);
    motor1.setAcceleration(ACCELERATION);
    motor2.setMaxSpeed(SPEED);
    motor2.setAcceleration(ACCELERATION);

    minElevation = calculateMinElevation();
    Serial.print("Obliczona minimalna elewacja: ");
    Serial.println(minElevation);
}

float TrackerMove::calculateMinElevation() {
    float L = MAX_SCREW_LENGTH - SAFETY_MARGIN;
    float sin_theta = 1.0 - pow(L / SOLAR_PANEL_R, 2) / 2.0;
    return 180.0 / PI * asin(sin_theta);
}
// Funkcja do homingu
// Ustawia silniki w pozycji zerowej - tracker w pozycji startowej
// Elewacja 90°, azymut 0°
void TrackerMove::homing() {
    Serial.println("Rozpoczynanie homingu elewacji...");
    motor1.setSpeed(-SPEED);
    digitalWrite(MOTOR1_EN_PIN, LOW);
    while (digitalRead(LIMIT_SWITCH_1) != LOW) {
        motor1.runSpeed();
    }
    motor1.stop();
    digitalWrite(MOTOR1_EN_PIN, HIGH);
    motor1.setCurrentPosition(0);
    currentElevation = 90.0;
    Serial.println("Homing elewacji zakończony");

    Serial.println("Rozpoczynanie homingu azymutu...");
    motor2.setSpeed(-SPEED);
    digitalWrite(MOTOR2_EN_PIN, LOW);
    while (digitalRead(LIMIT_SWITCH_2) != LOW) {
        motor2.runSpeed();
    }
    motor2.stop();
    digitalWrite(MOTOR2_EN_PIN, HIGH);
    motor2.setCurrentPosition(0);
    currentAzimuth = 0.0;
    Serial.println("Homing azymutu zakończony");
}
// Funkcja do ruchu azymutu
void TrackerMove::moveAzimuth(float targetAz) {
    targetAz = constrain(targetAz, 0.0, MAX_AZIMUTH);
    float diff = targetAz - currentAzimuth;
    long steps = (diff * WORM_GEAR_TEETH * STEPS_PER_REV) / 360.0;

    if (steps != 0) {
        Serial.print("Ruch azymutu: ");
        Serial.print(currentAzimuth);
        Serial.print("° -> ");
        Serial.print(targetAz);
        Serial.print("°, kroki: ");
        Serial.println(steps);

        digitalWrite(MOTOR2_EN_PIN, LOW);
        motor2.move(steps);

        while (motor2.distanceToGo() != 0) {
            motor2.run();
        }

        float movedDeg = (motor2.currentPosition() * 360.0) / (WORM_GEAR_TEETH * STEPS_PER_REV);
        currentAzimuth = constrain(currentAzimuth + movedDeg, 0.0, MAX_AZIMUTH);
        motor2.setCurrentPosition(0);
        digitalWrite(MOTOR2_EN_PIN, HIGH);

        Serial.print("Nowy azymut: ");
        Serial.println(currentAzimuth);
    }
}
// Funkcja do ruchu elewacji
void TrackerMove::moveElevation(float targetEl) {
    targetEl = constrain(targetEl, minElevation, 90.0);
    float currentLength = SOLAR_PANEL_R * sqrt(2 - 2 * sin(currentElevation * PI / 180.0));
    float targetLength = SOLAR_PANEL_R * sqrt(2 - 2 * sin(targetEl * PI / 180.0));
    float wormGearRatioEl = 1.0 / M5_LEAD;
    int stepsPerTurnEl = 200;
    long steps = (targetLength - currentLength) * wormGearRatioEl * stepsPerTurnEl;

    if (steps != 0) {
        Serial.print("Ruch elewacji: ");
        Serial.print(currentElevation);
        Serial.print("° -> ");
        Serial.print(targetEl);
        Serial.print("°, kroki: ");
        Serial.print(steps);
        Serial.print(", długość śruby: ");
        Serial.print(currentLength);
        Serial.print("mm -> ");
        Serial.print(targetLength);
        Serial.println("mm");

        digitalWrite(MOTOR1_EN_PIN, LOW);
        motor1.move(steps);

        while (motor1.distanceToGo() != 0) {
            motor1.run();
        }

        digitalWrite(MOTOR1_EN_PIN, HIGH);
        currentElevation = targetEl;

        Serial.print("Nowa elewacja: ");
        Serial.println(currentElevation);
    }
}
// Funkcja zwracająca aktualny azymut
float TrackerMove::getCurrentAzimuth() const {
    return currentAzimuth;
}
// Funkcja zwracająca aktualną elewację
float TrackerMove::getCurrentElevation() const {
    return currentElevation;
}
// Funkcja zwracająca minimalną elewację
float TrackerMove::getMinElevation() const {
    return minElevation;
}
// Funkcja wykonująca ruch trackerem
void TrackerMove::moveTracker(float targetAz, float targetEl) {
    Serial.print("Ustawiam panel - Azymut: ");
    Serial.print(targetAz);
    Serial.print("°, Elewacja: ");
    Serial.print(targetEl);
    Serial.print("° (min elewacja: ");
    Serial.print(this->getMinElevation());
    Serial.println("°)");

    this->moveAzimuth(targetAz);
    this->moveElevation(targetEl);
}
