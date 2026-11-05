#include <Arduino.h>

#include "line_follower.hpp"
#include "motor.h"
#include "config.h"


// ======================================================
// INTERNAL STATE
// ======================================================

static float filteredPosition = 0.0f;
static float lastError = 0.0f;

static bool hasPreviousSample = false;

enum class LastSeenSide {
    UNKNOWN,
    LEFT,
    CENTER,
    RIGHT
};

static LastSeenSide lastSeenSide = LastSeenSide::UNKNOWN;
static bool recovering = false;
static unsigned long recoveryStartMs = 0; //mulai recovery jam brp

// HITUNG POSISI GARIS
static bool calculateLinePosition(const BFDReading &reading,float &position) {
    float weightedSum = 0.0f;
    int activeCount = 0;

    // Posisi sensor dari kiri ke kanan:
    //
    // LO      LI      RI      RO
    // -1.5    -0.5    +0.5    +1.5

    if (reading.leftOuter) {
        weightedSum += -1.5f;
        activeCount++;
    }

    if (reading.leftInner) {
        weightedSum += -0.5f;
        activeCount++;
    }

    if (reading.rightInner) {
        weightedSum += 0.5f;
        activeCount++;
    }

    if (reading.rightOuter) {
        weightedSum += 1.5f;
        activeCount++;
    }

    // Tidak ada sensor belakang yang melihat garis.
    if (activeCount == 0) {
        return false;
    }
    position = weightedSum/activeCount;
    return true;
}

// BASE SPEED
static int calculateBaseSpeed(const BFDReading &reading,float error) {

    bool outerActive = reading.leftOuter || reading.rightOuter;

    // Klo garis sudah menyentuh sensor paling luar, anggap robot error besar -> perlambat.
    int effectiveBase = outerActive ? LF_CURVE_SPEED : LF_BASE_SPEED;

    // Semakin jauh dari tengah, semakin pelan robot berjalan.
    int baseSpeed = effectiveBase - (int)(fabsf(error) *LF_SPEED_DROP);
    return constrain(baseSpeed,LF_MIN_SPEED,effectiveBase);
}

void lineFollowerInit() {
    lineFollowerReset();
}

void lineFollowerReset() {
    filteredPosition = 0.0f;
    lastError = 0.0f;
    hasPreviousSample = false;
    lastSeenSide = LastSeenSide::UNKNOWN;
    recovering = false;
    recoveryStartMs = 0;
}

static void updateLastSeenSide(float error) {
    if (error < -0.1f) {
        lastSeenSide = LastSeenSide::LEFT;
    }
    else if (error > 0.1f) {
        lastSeenSide = LastSeenSide::RIGHT;
    }
    else {
        lastSeenSide = LastSeenSide::CENTER;
    }
}

static void runShortRecovery() {

    if (lastSeenSide == LastSeenSide::LEFT) {
        drive(LF_RECOVERY_SLOW_SPEED,LF_RECOVERY_FAST_SPEED);
    }

    else if (lastSeenSide == LastSeenSide::RIGHT
    ) {
        drive(LF_RECOVERY_FAST_SPEED,LF_RECOVERY_SLOW_SPEED);
    }

    else {
        drive(LF_RECOVERY_SLOW_SPEED,LF_RECOVERY_SLOW_SPEED);
    }
}

LineFollowStatus lineFollowerUpdate(const BFDReading &reading) {

    float rawPosition = 0.0f;
    bool lineDetected = calculateLinePosition(reading, rawPosition);
    if (!lineDetected) {
        if(!recovering){
            recovering  = true;
            recoveryStartMs = millis();
            hasPreviousSample = false; //biar pas udh ketemu line, derivative ga make data lama
        }
        if(millis() - recoveryStartMs < LF_SHORT_RECOVERY_MS){
            runShortRecovery();
            return LineFollowStatus::RECOVERING;
        }
        stopAllMotors();
        recovering = false;
        return LineFollowStatus::LOST;
    }
    recovering = false; // klo tadi lg recovery, brti skrng udh ketemu line

    if (!hasPreviousSample) {
        // Sampel pertama ga ngitung derivative dlu.
        filteredPosition = rawPosition;
    }
    else {
        filteredPosition = LF_FILTER_ALPHA *filteredPosition +(1.0f - LF_FILTER_ALPHA) *rawPosition; //biar ga terlalu drastis
    }
    float error = filteredPosition;
    float derivative = 0.0f;
    updateLastSeenSide(error);

    if (hasPreviousSample) {
        derivative = error -lastError;
    }

    // PD CORRECTION
    float correction = LF_KP * error + LF_KD * derivative;
    correction =constrain(correction,-LF_MAX_CORRECTION,LF_MAX_CORRECTION );


    int baseSpeed =calculateBaseSpeed(reading, error);

    int leftSpeed =baseSpeed + (int)correction;
    int rightSpeed = baseSpeed - (int)correction;

    leftSpeed =constrain(leftSpeed,0,PWM_MAX);
    rightSpeed =constrain(rightSpeed,0,PWM_MAX);

    drive(leftSpeed,rightSpeed);

    //simpan state
    lastError =error;
    hasPreviousSample =true;
    return LineFollowStatus::TRACKING;
}