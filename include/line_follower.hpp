#pragma once

#include "bfd.hpp"

enum class LineFollowStatus {
    TRACKING,
    RECOVERING,
    LOST
};


void lineFollowerInit();

void lineFollowerReset(); //biar setelah pivot bisa reset lastError

LineFollowStatus lineFollowerUpdate(const BFDReading &reading); //linefollowing biasa atau harus ke intersection.cpp