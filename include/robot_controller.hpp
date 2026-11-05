#pragma once

#include <stdint.h>

enum class RobotMode : uint8_t {
    IDLE,
    MANUAL_HARVEST,
    AUTO_MAZE,
    FINISHED,
    ERROR
};

void robotControllerInit();
void robotControllerUpdate();

void robotControllerSetMode(RobotMode newMode);
RobotMode robotControllerGetMode();

bool robotControllerManualDrive(int leftSpeed, int rightSpeed);

const char* robotModeToString(RobotMode mode);