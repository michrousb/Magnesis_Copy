#pragma once
#include <Arduino.h>

enum MotorID {
    FRONT_LEFT = 0,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT
};

void motorInit();
void motorSet(MotorID motor, int speed);

void motorCoast(MotorID motor);
void motorBrake(MotorID motor);

void stopAllMotors();
void drive(int leftSpeed, int rightSpeed);

void forward(int speed);
void backward(int speed);

void turnLeft(int speed);
void turnRight(int speed);

void pivotLeft(int speed);
void pivotRight(int speed);
