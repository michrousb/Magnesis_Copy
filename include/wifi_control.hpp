#pragma once

#include <Arduino.h>
#include <stdint.h>

// Aksi dari tombol web. BUKAN RobotMode: mode tetap dikelola robot_controller.
enum class WifiButton : uint8_t {
    NONE,
    MANUAL,
    AUTO,
    HARVEST,
    STOP
};

// Snapshot joystick terbaru, bukan state motor.
struct WifiJoystickState {
    float x = 0.0f;             // -7 .. +7, positif = kanan
    float y = 0.0f;             // -7 .. +7, positif = maju
    unsigned long receivedAt = 0;
    uint32_t sequence = 0;      // bertambah setiap paket joystick valid
    bool hasReceived = false;
};

bool wifiControlInit();
void wifiControlUpdate();
WifiJoystickState wifiControlGetJoystick();
WifiButton wifiControlPopButton();
void wifiControlSetModeLabel(const char* modeLabel);
