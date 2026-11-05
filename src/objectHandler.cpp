
#include <Arduino.h>
#include "objectHandler.hpp"
#include "config.h"


static bool conveyorRunning = false;

void conveyorInit() {
    pinMode(CONVEYOR_IN2, OUTPUT);
    digitalWrite(CONVEYOR_IN2, LOW);

    ledcSetup(
        CONVEYOR_PWM_CHANNEL,
        CONVEYOR_PWM_FREQ,
        CONVEYOR_PWM_RESOLUTION
    );

    ledcAttachPin(CONVEYOR_IN1, CONVEYOR_PWM_CHANNEL);

    conveyorStop();
}

void conveyorRun(uint8_t speed) {
    digitalWrite(CONVEYOR_IN2, LOW);
    ledcWrite(CONVEYOR_PWM_CHANNEL, speed);

    conveyorRunning = (speed > 0);
}

void conveyorStop() {
    ledcWrite(CONVEYOR_PWM_CHANNEL, 0);
    digitalWrite(CONVEYOR_IN2, LOW);

    conveyorRunning = false;
}

bool conveyorIsRunning() {
    return conveyorRunning;
}