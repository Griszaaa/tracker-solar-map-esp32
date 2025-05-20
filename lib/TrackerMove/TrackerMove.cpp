#include <TrackerMove.h>
#include <WiFiLogger.h> 
#include <Arduino.h>
#include <math.h>

TrackerMove::TrackerMove()
    : motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN),
      motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN),
      currentAzimuth(0.0), currentElevation(90.0), minElevation(0.0) {}

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
    Logger.print("Obliczona minimalna elewacja: ");
    Logger.println(String(minElevation));
}

float TrackerMove::calculateMinElevation() {
    float L = MAX_SCREW_LENGTH - SAFETY_MARGIN;
    float sin_theta = 1.0 - pow(L / SOLAR_PANEL_R, 2) / 2.0;
    return 180.0 / PI * asin(sin_theta);
}

void TrackerMove::homing() {
    Logger.println("Rozpoczynanie homingu elewacji...");
    motor1.setSpeed(-SPEED);
    digitalWrite(MOTOR1_EN_PIN, LOW);
    while (digitalRead(LIMIT_SWITCH_1) != LOW) {
        motor1.runSpeed();
    }
    motor1.stop();
    digitalWrite(MOTOR1_EN_PIN, HIGH);
    motor1.setCurrentPosition(0);
    currentElevation = 90.0;
    Logger.println("Homing elewacji zakończony");

    Logger.println("Rozpoczynanie homingu azymutu...");
    motor2.setSpeed(-SPEED);
    digitalWrite(MOTOR2_EN_PIN, LOW);
    while (digitalRead(LIMIT_SWITCH_2) != LOW) {
        motor2.runSpeed();
    }
    motor2.stop();
    digitalWrite(MOTOR2_EN_PIN, HIGH);
    motor2.setCurrentPosition(0);
    currentAzimuth = 0.0;
    Logger.println("Homing azymutu zakończony");
}

void TrackerMove::moveAzimuth(float targetAz) {
    targetAz = constrain(targetAz, 0.0, MAX_AZIMUTH);
    float diff = targetAz - currentAzimuth;
    long steps = (diff * WORM_GEAR_TEETH * STEPS_PER_REV) / 360.0;

    if (steps != 0) {
        Logger.print("Ruch azymutu: ");
        Logger.print(String(currentAzimuth));
        Logger.print("° -> ");
        Logger.print(String(targetAz));
        Logger.print("°, kroki: ");
        Logger.println(String(steps));

        digitalWrite(MOTOR2_EN_PIN, LOW);
        motor2.move(steps);

        while (motor2.distanceToGo() != 0) {
            motor2.run();
        }

        float movedDeg = (motor2.currentPosition() * 360.0) / (WORM_GEAR_TEETH * STEPS_PER_REV);
        currentAzimuth = constrain(currentAzimuth + movedDeg, 0.0, MAX_AZIMUTH);
        motor2.setCurrentPosition(0);
        digitalWrite(MOTOR2_EN_PIN, HIGH);

        Logger.print("Nowy azymut: ");
        Logger.println(String(currentAzimuth));
    }
}

void TrackerMove::moveElevation(float targetEl) {
    targetEl = constrain(targetEl, minElevation, 90.0);
    float currentLength = SOLAR_PANEL_R * sqrt(2 - 2 * sin(currentElevation * PI / 180.0));
    float targetLength = SOLAR_PANEL_R * sqrt(2 - 2 * sin(targetEl * PI / 180.0));
    float wormGearRatioEl = 1.0 / M5_LEAD;
    int stepsPerTurnEl = 200;
    long steps = (targetLength - currentLength) * wormGearRatioEl * stepsPerTurnEl;

    if (steps != 0) {
        Logger.print("Ruch elewacji: ");
        Logger.print(String(currentElevation));
        Logger.print("° -> ");
        Logger.print(String(targetEl));
        Logger.print("°, kroki: ");
        Logger.print(String(steps));
        Logger.print(", długość śruby: ");
        Logger.print(String(currentLength));
        Logger.print("mm -> ");
        Logger.print(String(targetLength));
        Logger.println("mm");

        digitalWrite(MOTOR1_EN_PIN, LOW);
        motor1.move(steps);

        while (motor1.distanceToGo() != 0) {
            motor1.run();
        }

        digitalWrite(MOTOR1_EN_PIN, HIGH);
        currentElevation = targetEl;

        Logger.print("Nowa elewacja: ");
        Logger.println(String(currentElevation));
    }
}

float TrackerMove::getCurrentAzimuth() const {
    return currentAzimuth;
}

float TrackerMove::getCurrentElevation() const {
    return currentElevation;
}

float TrackerMove::getMinElevation() const {
    return minElevation;
}

void TrackerMove::moveTracker(float targetAz, float targetEl) {
    Logger.print("Ustawiam panel - Azymut: ");
    Logger.print(String(targetAz));
    Logger.print("°, Elewacja: ");
    Logger.print(String(targetEl));
    Logger.print("° (min elewacja: ");
    Logger.print(String(this->getMinElevation()));
    Logger.println("°)");

    this->moveAzimuth(targetAz);
    this->moveElevation(targetEl);
}
