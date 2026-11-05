#include <Arduino.h>

#include "robot_controller.hpp"
#include "motor.h"
#include "navigation.hpp"
#include "maze.hpp"

static RobotMode currentMode = RobotMode::IDLE;

static constexpr unsigned long MANUAL_TIMEOUT_MS = 300;
static unsigned long lastManualCommandTime = 0;
static bool manualCommandActive = false;


// debug
const char* robotModeToString(RobotMode mode) {
    switch (mode) {
        case RobotMode::IDLE:
            return "IDLE";

        case RobotMode::MANUAL_HARVEST:
            return "MANUAL_HARVEST";

        case RobotMode::AUTO_MAZE:
            return "AUTO_MAZE";

        case RobotMode::FINISHED:
            return "FINISHED";

        case RobotMode::ERROR:
            return "ERROR";
    }

    return "UNKNOWN";
}


// Inisialisasi controller
void robotControllerInit() {
    stopAllMotors();

    currentMode = RobotMode::IDLE;
    manualCommandActive = false;
    lastManualCommandTime = 0;

    Serial.println("[ROBOT] Controller initialized");
}


// Mengganti mode robot
void robotControllerSetMode(RobotMode newMode) {
    if (currentMode == newMode) {
        return;
    }
    if (currentMode == RobotMode::ERROR && newMode != RobotMode::IDLE) {
        Serial.println("[ROBOT] Reset to IDLE first!");
        return;
    }
    stopAllMotors();
    manualCommandActive = false;

    if (newMode == RobotMode::AUTO_MAZE) {
        navigationInit();
    }

    currentMode = newMode;

    Serial.print("[ROBOT] Mode: ");
    Serial.println(robotModeToString(currentMode));
}

RobotMode robotControllerGetMode() {
    return currentMode;
}

// Perintah menggerakkan robot secara manual
bool robotControllerManualDrive(int leftSpeed, int rightSpeed) {
    if (currentMode != RobotMode::MANUAL_HARVEST) {
        return false;
    }

    drive(leftSpeed, rightSpeed);

    lastManualCommandTime = millis();
    manualCommandActive = true;

    return true;
}


// Update controller
void robotControllerUpdate() {
    switch (currentMode) {

        case RobotMode::IDLE: {
            stopAllMotors();
            return;
        }

        case RobotMode::MANUAL_HARVEST: {
            // Berhenti jika perintah manual tidak diperbarui
            if (manualCommandActive &&
                millis() - lastManualCommandTime >= MANUAL_TIMEOUT_MS) {

                stopAllMotors();
                manualCommandActive = false;

                Serial.println("[MANUAL] Command timeout, motors stopped");
            }

            return;
        }

        case RobotMode::AUTO_MAZE: {
            navigationUpdate();

            if (navigationHasError()) {
                robotControllerSetMode(RobotMode::ERROR);
                return;
            }

            if (navigationIsFinished()) {
                if (mazeGoalReached()) {
                    Serial.println("[ROBOT] Maze mission completed");
                    robotControllerSetMode(RobotMode::MANUAL_HARVEST);
                } else {
                    Serial.println("[ROBOT] Exploration ended without goal");
                    robotControllerSetMode(RobotMode::IDLE);
                }
                return;
            }

            return;
        }


        case RobotMode::FINISHED: {
            stopAllMotors();
            return;
        }

        case RobotMode::ERROR: {
            stopAllMotors();
            return;
        }
    }
}