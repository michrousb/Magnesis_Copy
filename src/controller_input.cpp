#include <Arduino.h>
#include <math.h>

#include "controller_input.hpp"
#include "wifi_control.hpp"
#include "robot_controller.hpp"
#include "objectHandler.hpp"
#include "config.h"

namespace {

// Rentang joystick dari halaman kontrol HP (seperti versi Dabble).
constexpr float JOYSTICK_MAX = 7.0f;
constexpr float DEADZONE = 0.8f;
constexpr int MAX_MANUAL_PWM = 160;

// robot_controller.cpp menghentikan motor setelah 300 ms tanpa perintah baru.
// Setelah 400 ms tanpa paket joystick Wi-Fi, keluar dari mode MANUAL.
constexpr unsigned long RX_TIMEOUT_MS = 400;

// TRUE  = tes input HP + Serial Monitor, tanpa menggerakkan motor manual.
//         Tombol AUTO juga dinonaktifkan selama pengujian.
// FALSE = joystick benar-benar menggerakkan motor; AUTO diizinkan.
constexpr bool CONTROLLER_TEST_ONLY = true;
constexpr unsigned long DEBUG_PRINT_INTERVAL_MS = 200;

uint32_t lastProcessedJoystickSequence = 0;
RobotMode lastObservedMode = RobotMode::IDLE;
bool needsNeutralBeforeDriving = true;
bool wasFresh = false;
unsigned long lastDebugPrintMs = 0;

float applyDeadzone(float value) {
    return (fabsf(value) < DEADZONE) ? 0.0f : value;
}

bool joystickIsNeutral(const WifiJoystickState& joystick) {
    return applyDeadzone(joystick.x) == 0.0f &&
           applyDeadzone(joystick.y) == 0.0f;
}

void observeModeChange() {
    const RobotMode mode = robotControllerGetMode();

    if (mode != lastObservedMode) {
        // Termasuk AUTO_MAZE -> MANUAL_HARVEST setelah navigasi selesai.
        // Wajib netralkan joystick sebelum motor manual boleh bergerak.
        needsNeutralBeforeDriving = (mode == RobotMode::MANUAL_HARVEST);
        lastObservedMode = mode;
    }
}

const char* getDirection(float x, float y) {
    if (x == 0.0f && y == 0.0f) return "STOP";

    if (y > 0.0f) {
        if (x > 0.0f) return "MAJU KANAN";
        if (x < 0.0f) return "MAJU KIRI";
        return "MAJU";
    }

    if (y < 0.0f) {
        if (x > 0.0f) return "MUNDUR KANAN";
        if (x < 0.0f) return "MUNDUR KIRI";
        return "MUNDUR";
    }

    return (x > 0.0f) ? "PUTAR KANAN" : "PUTAR KIRI";
}

void printJoystickDebug(float x, float y,
                        int leftSpeed, int rightSpeed,
                        bool waitingForNeutral) {
    if (millis() - lastDebugPrintMs < DEBUG_PRINT_INTERVAL_MS) {
        return;
    }

    lastDebugPrintMs = millis();

    Serial.printf(
        "[%s] X=%.2f Y=%.2f | LEFT=%d RIGHT=%d | %s | %s\n",
        CONTROLLER_TEST_ONLY ? "TEST" : "JOYSTICK",
        x,
        y,
        leftSpeed,
        rightSpeed,
        getDirection(x, y),
        waitingForNeutral ? "WAIT_NEUTRAL" : "READY"
    );
}

} // namespace

void controllerInputInit() {
    lastProcessedJoystickSequence = 0;
    lastObservedMode = robotControllerGetMode();
    needsNeutralBeforeDriving = true;
    wasFresh = false;
    lastDebugPrintMs = 0;

    if (wifiControlInit()) {
        Serial.println("[CONTROLLER] Wi-Fi input ready");
    } else {
        Serial.println("[CONTROLLER] Wi-Fi AP failed; robot stays safe");
    }

    if (CONTROLLER_TEST_ONLY) {
        Serial.println("[TEST] Controller test enabled");
        Serial.println("[TEST] Manual motor output = 0; AUTO disabled");
        Serial.println("[TEST] LEFT/RIGHT shown are calculated PWM commands");
    }

    wifiControlSetModeLabel(robotModeToString(robotControllerGetMode()));
}

void controllerInputUpdate() {
    // Tetap dipanggil sering dari loop() agar web server responsif.
    wifiControlUpdate();

    const WifiJoystickState joystick = wifiControlGetJoystick();
    const bool fresh = joystick.hasReceived &&
        (millis() - joystick.receivedAt <= RX_TIMEOUT_MS);

    // Tombol adalah event terpisah dari paket joystick.
    bool stopRequested = false;
    WifiButton button = WifiButton::NONE;

    while ((button = wifiControlPopButton()) != WifiButton::NONE) {
        if (button == WifiButton::STOP) {
            robotControllerSetMode(RobotMode::IDLE);
            stopRequested = true;
            Serial.println("[WIFI] STOP pressed");
            continue;
        }

        // STOP mengalahkan tombol lain pada siklus update yang sama.
        if (stopRequested) {
            continue;
        }

        const RobotMode mode = robotControllerGetMode();

        if (button == WifiButton::MANUAL && mode == RobotMode::IDLE) {
            if (fresh) {
                robotControllerSetMode(RobotMode::MANUAL_HARVEST);
                Serial.println("[WIFI] Mode -> MANUAL_HARVEST");
            } else {
                Serial.println("[WIFI] Wait for fresh joystick data");
            }
        } else if (button == WifiButton::AUTO &&
                   mode == RobotMode::MANUAL_HARVEST) {
            if (CONTROLLER_TEST_ONLY) {
                Serial.println("[TEST] AUTO disabled during controller testing");
            } else if (fresh && joystickIsNeutral(joystick)) {
                // TODO: cek posisi sensor BFD sebelum mulai AUTO_MAZE.
                robotControllerSetMode(RobotMode::AUTO_MAZE);
                Serial.println("[WIFI] Mode -> AUTO_MAZE");
            } else {
                Serial.println("[WIFI] Release joystick before AUTO");
            }
        }
            else if (button == WifiButton::HARVEST &&
                    mode == RobotMode::MANUAL_HARVEST) {

                if (CONTROLLER_TEST_ONLY) {
                    Serial.println("[TEST] HARVEST disabled during testing");
                }
                else if (conveyorIsRunning()) {
                    conveyorStop();
                    Serial.println("[HARVEST] Conveyor OFF");
                }
                else {
                    conveyorRun(CONVEYOR_DEFAULT_SPEED);
                    Serial.println("[HARVEST] Conveyor ON");
                }
            }

    }

    observeModeChange();

    if (robotControllerGetMode() == RobotMode::MANUAL_HARVEST) {
        if (!fresh) {
            // Failsafe: motor manual berhenti setelah 300 ms (robot_controller),
            // lalu mode ditutup setelah 400 ms tanpa paket joystick baru.
            robotControllerSetMode(RobotMode::IDLE);
            if (wasFresh) {
                Serial.println("[WIFI] Signal timeout: MANUAL -> IDLE");
            }
            observeModeChange();
        } else if (joystick.sequence != lastProcessedJoystickSequence) {
            // Hanya paket BARU yang boleh me-refresh watchdog manual.
            lastProcessedJoystickSequence = joystick.sequence;

            const float x = applyDeadzone(joystick.x);
            const float y = applyDeadzone(joystick.y);

            if (needsNeutralBeforeDriving) {
                if (x == 0.0f && y == 0.0f) {
                    needsNeutralBeforeDriving = false;
                }

                // Tetap berhenti sampai joystick pernah netral.
                robotControllerManualDrive(0, 0);
                printJoystickDebug(x, y, 0, 0, needsNeutralBeforeDriving);
            } else {
                // Differential drive: Y = maju/mundur, X = belok.
                const float leftMix = y + x;
                const float rightMix = y - x;

                const float maxMagnitude =
                    fmaxf(fabsf(leftMix), fabsf(rightMix));
                const float scale = fmaxf(JOYSTICK_MAX, maxMagnitude);

                const int leftSpeed = static_cast<int>(roundf(
                    leftMix / scale * MAX_MANUAL_PWM));
                const int rightSpeed = static_cast<int>(roundf(
                    rightMix / scale * MAX_MANUAL_PWM));

                // Saat TEST_ONLY, nilai PWM hanya dicetak; motor menerima 0.
                const bool accepted = robotControllerManualDrive(
                    CONTROLLER_TEST_ONLY ? 0 : leftSpeed,
                    CONTROLLER_TEST_ONLY ? 0 : rightSpeed
                );

                if (!accepted) {
                    Serial.println("[WIFI] Manual command rejected");
                }

                printJoystickDebug(x, y, leftSpeed, rightSpeed, false);
            }
        }
    } else {
        // Abaikan joystick di luar mode MANUAL.
        lastProcessedJoystickSequence = joystick.sequence;
    }

    wasFresh = fresh;
    wifiControlSetModeLabel(robotModeToString(robotControllerGetMode()));
}
