#include <Arduino.h>

#include "motor.h"
#include "encoder.h"
#include "bfd.hpp"

#include "robot_controller.hpp"
#include "controller_input.hpp"


void setup() {
    Serial.begin(115200);

    motorInit();
    encoderInit();
    bfdInit();

    robotControllerInit();
    controllerInputInit();

    Serial.println("[MAGNESIS] Ready!");
}


void loop() {
    controllerInputUpdate();

    robotControllerUpdate();

    delay(1);
}