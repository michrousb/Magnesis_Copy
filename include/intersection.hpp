#pragma once

#include "bfd.hpp"


enum class JunctionType {
    NORMAL,
    LEFT_ONLY,
    RIGHT_ONLY,
    STRAIGHT_ONLY,
    LEFT_STRAIGHT,
    RIGHT_STRAIGHT,
    T_JUNCTION,
    CROSS,
    DEAD_END,
    UNKNOWN
};


struct JunctionInfo { //kesimpulan ada jalan kiri/lurus/kanan/balik
    bool left;
    bool straight;
    bool right;
    bool back;

    JunctionType type;
};


bool intersectionCandidate(const BFDReading &reading);

void intersectionScanReset();

void intersectionScanObserve(const BFDReading &reading);

JunctionInfo intersectionScanFinish(bool hasBack);

const char* junctionTypeToString(JunctionType type); //buat debug ae