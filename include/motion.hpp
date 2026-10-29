#pragma once

//motion ini buat gabungin motor + encoder untuk gerakan tertenut

void moveForwardTicks(int speed, long targetTicks);

void turnLeft90();

void turnRight90();

void turnAround180();

void pivtoTicks( bool turnLeftDirection, long targetTicks);

void motionResetProgress();

long motionGetLeftTicks();

long motionGetRightTicks();

bool motionTargetReached(long targetTicks);