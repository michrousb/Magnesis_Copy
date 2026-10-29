#include <Arduino.h>

#include "bfd.hpp"
#include "config.h"

void bfdInit() {

    pinMode(BFD_FRONT, INPUT);
    pinMode(BFD_LEFT_INNER, INPUT);
    pinMode(BFD_LEFT_OUTER, INPUT);
    pinMode(BFD_RIGHT_INNER, INPUT);
    pinMode(BFD_RIGHT_OUTER, INPUT);
}

BFDReading bfdRead() {

    BFDReading reading;

    reading.front =
        digitalRead(BFD_FRONT) == BFD_ACTIVE_LEVEL; // misal low == low berarti true -> ada garis

    reading.leftOuter =
        digitalRead(BFD_LEFT_OUTER) == BFD_ACTIVE_LEVEL;

    reading.leftInner =
        digitalRead(BFD_LEFT_INNER) == BFD_ACTIVE_LEVEL;

    reading.rightInner =
        digitalRead(BFD_RIGHT_INNER) == BFD_ACTIVE_LEVEL;

    reading.rightOuter =
        digitalRead(BFD_RIGHT_OUTER) == BFD_ACTIVE_LEVEL;


    reading.activeCount =
        reading.front +
        reading.leftInner +
        reading.leftOuter+
        reading.rightInner+
        reading.rightOuter;


    return reading;
}

void bfdPrint(const BFDReading &reading) {

    Serial.print("FL");
    Serial.print(reading.front);

    Serial.print(" | LO =");
    Serial.print(reading.leftOuter);

    Serial.print("LI=");
    Serial.print(reading.leftInner);

    Serial.print("RI=");
    Serial.print(reading.rightInner);

    Serial.print("RO=");
    Serial.print(reading.rightOuter);

    Serial.print(" | active=");
    Serial.println(reading.activeCount);
}