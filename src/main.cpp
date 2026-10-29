/**
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE

#include <DabbleESP32.h>


void setup() {
    Serial.begin(115200);

    motorInit();
    encoderInit();

    Dabble.begin("RobotMagnesis");
}
if (GamePad.isUpPressed()) {
    forward(150);
}

if (GamePad.isDownPressed()) {
    backward(150);
}

if (GamePad.isLeftPressed()) {
    turnLeft(150);
}

if (GamePad.isRightPressed()) {
    turnRight(150);
}
    */