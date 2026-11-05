#include <Arduino.h>
#include "motion.hpp"
#include "motor.h"
#include "encoder.h"
#include "config.h"

static long getLeftTicks(){
    long frontLeft = abs(encoderGetCount(ENCODER_FRONT_LEFT));
    long rearLeft = abs(encoderGetCount(ENCODER_REAR_LEFT));
    return(frontLeft + rearLeft)/2;
}

static long getRightTicks(){
    long frontRight = abs(encoderGetCount(ENCODER_FRONT_RIGHT));
    long rearRight = abs(encoderGetCount(ENCODER_REAR_RIGHT));
    return(frontRight + rearRight)/2;
}

bool moveForwardTicks(int speed, long targetTicks){
    if (targetTicks <= 0 || speed == 0) {
        stopAllMotors();
        return false;
    }
    speed = abs(speed);
    encoderResetAll();
    unsigned long startTime = millis(); 
    forward(speed);
    while(true){
        long leftTicks = getLeftTicks();
        long rightTicks = getRightTicks();
        bool leftDone = leftTicks >= targetTicks;
        bool rightDone = rightTicks >= targetTicks;
        int rightSpeed = rightDone ? 0 : speed;
        int leftSpeed = leftDone ? 0 : speed;
        drive(leftSpeed, rightSpeed);
        if(leftDone && rightDone){
            stopAllMotors();
            return true;
        }
        if(millis() - startTime >= MOTION_TIMEOUT_MS){
            stopAllMotors();
            return false;
        }
    }
}

static bool pivotTicks(bool turnLeftDirection, long targetTicks){
    if (targetTicks <= 0 || MOTION_TURN_SPEED <= 0) {
        stopAllMotors();
        return false;
    }
    encoderResetAll();
    unsigned long startTime = millis();
    while(true){
        long leftTicks = getLeftTicks();
        long rightTicks = getRightTicks();
        bool leftDone = leftTicks >= targetTicks;
        bool rightDone = rightTicks >= targetTicks;
        int leftSpeed;
        int rightSpeed;
        if(turnLeftDirection){ //klo false berarti kanan
            leftSpeed = leftDone ? 0 : -MOTION_TURN_SPEED;
            rightSpeed = rightDone ? 0 : MOTION_TURN_SPEED;
        }
        else{
            leftSpeed = leftDone ? 0 : MOTION_TURN_SPEED;
            rightSpeed = rightDone ? 0 : -MOTION_TURN_SPEED;
            
        }
        drive(leftSpeed, rightSpeed);
        
        if(leftDone && rightDone){
            stopAllMotors();
            return true;
        }
        if(millis() - startTime >= MOTION_TIMEOUT_MS){
            stopAllMotors();
            Serial.println("Motion timeout!");
            return false;
        }
        delay(1);
    }

}

bool turnLeft90(){
    return pivotTicks(true, TURN_90_TICKS);
}

bool  turnRight90(){
    return pivotTicks(false, TURN_90_TICKS);
}

bool turnAround180(){
    return pivotTicks(false, TURN_180_TICKS); 
}

void motionResetProgress() {
    encoderResetAll();
}

long motionGetLeftTicks() {
    return getLeftTicks();
}

long motionGetRightTicks() {
    return getRightTicks();
}

bool motionTargetReached(long targetTicks) {
    long leftTicks = getLeftTicks();
    long rightTicks = getRightTicks();

    return (leftTicks >= targetTicks && rightTicks >= targetTicks);
}