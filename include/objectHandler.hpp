#pragma once
#include <Arduino.h>
#include <stdint.h>

void conveyorInit();
void conveyorRun(uint8_t speed);
void conveyorStop();
bool conveyorIsRunning();