#pragma once

#include <Arduino.h>

struct BFDReading{
    bool leftOuter; //true = garis terdeteksi, false = garis tidak terdeteksi
    bool leftInner;
    bool rightInner;
    bool rightOuter;
    bool front;
    int activeCount; //jmlh sensor yang sedang true
};

void bfdInit();

BFDReading bfdRead();

void bfdPrint(const BFDReading &reading); //buat debugging

bool bfdAllBlack(const BFDReading &reading);