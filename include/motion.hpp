#pragma once

//motion ini buat gabungin motor + encoder untuk gerakan tertenut

bool moveForwardTicks(int speed, long targetTicks);

bool turnLeft90();

bool turnRight90();

bool turnAround180();

void motionResetProgress();

long motionGetLeftTicks();

long motionGetRightTicks();

bool motionTargetReached(long targetTicks);