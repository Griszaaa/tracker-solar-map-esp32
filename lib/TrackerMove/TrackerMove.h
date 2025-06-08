#ifndef TRACKER_MOVE_H
#define TRACKER_MOVE_H

#include <AccelStepper.h>

// Definicje stałych i pinów
// Silnik 1 – Elewacja
#define MOTOR1_DIR_PIN    17
#define MOTOR1_STEP_PIN   16
#define MOTOR1_EN_PIN     4

// Silnik 2 – Azymut
#define MOTOR2_DIR_PIN    19
#define MOTOR2_STEP_PIN   18
#define MOTOR2_EN_PIN     5

// Krańcówki
#define LIMIT_SWITCH_1    15  // Krańcówka elewacji
#define LIMIT_SWITCH_2    2   // Krańcówka azymutu


#define MOTOR_STEPS 200
#define STEPS_PER_REV MOTOR_STEPS
#define SPEED_IN_RPS 3.0
#define SPEED (SPEED_IN_RPS * STEPS_PER_REV)
#define ACCELERATION SPEED
#define WORM_GEAR_TEETH 44.0
#define MAX_AZIMUTH 360.0
#define SOLAR_PANEL_R 330.0
#define MAX_SCREW_LENGTH 340.0
#define M5_LEAD 0.8
#define SAFETY_MARGIN 2.0

class TrackerMove {
public:
    TrackerMove();
    void begin();
    void homing();
    void moveAzimuth(float targetAz);
    void moveElevation(float targetEl);
    float getCurrentAzimuth() const;
    float getCurrentElevation() const;
    float getMinElevation() const;
    void moveTracker(float targetAz, float targetEl);

private:
    AccelStepper motor1;
    AccelStepper motor2;
    float currentAzimuth;
    float currentElevation;
    float minElevation;

    float calculateMinElevation();
};

#endif
