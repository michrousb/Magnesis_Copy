#include <Arduino.h>
#include "motor.h"
#include "config.h"

struct MotorConfig {
    uint8_t in1;
    uint8_t in2;
    bool inverted;
};

static const MotorConfig motors[4] = {
    {FL_IN1, FL_IN2, FL_INVERTED},
    {FR_IN1, FR_IN2, FR_INVERTED},
    {RL_IN1, RL_IN2, RL_INVERTED},
    {RR_IN1, RR_IN2, RR_INVERTED}
};

/*motors[0] = Front Left
motors[1] = Front Right
motors[2] = Rear Left
motors[3] = Rear Right
*/

// in1 aktif = arah positif (maju)
// in2 aktif = arah negatif (mundur)
void motorInit() {
    for (int i = 0; i < 4; i++) {
        int channel1 = i * 2;
        int channel2 = i * 2 + 1;
        ledcSetup(channel1, PWM_FREQ, PWM_RESOLUTION);
        ledcSetup(channel2, PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(motors[i].in1, channel1);
        ledcAttachPin(motors[i].in2, channel2);
        ledcWrite(channel1, 0);
        ledcWrite(channel2, 0);
    }
}


void motorSet(MotorID motor, int speed) {
    speed = constrain(speed, -PWM_MAX, PWM_MAX);

    const int index = static_cast<int>(motor);
    const int channel1 = index * 2;
    const int channel2 = index * 2 + 1;

    if (motors[index].inverted) {
        speed = -speed;
    }

    if (speed > 0) {
        ledcWrite(channel1, speed);
        ledcWrite(channel2, 0);
    }
    else if (speed < 0) {
        ledcWrite(channel1, 0);
        ledcWrite(channel2, -speed);
    }
    else {
        ledcWrite(channel1, 0);
        ledcWrite(channel2, 0);
    }
}


void drive(int leftSpeed, int rightSpeed) {
    motorSet(FRONT_LEFT, leftSpeed);
    motorSet(REAR_LEFT, leftSpeed);

    motorSet(FRONT_RIGHT, rightSpeed);
    motorSet(REAR_RIGHT, rightSpeed);
}

void forward(int speed) {
    speed = abs(speed);
    drive(speed, speed);
    Serial.print("FORWARD, speed = ");
}

void backward(int speed) {
    speed = abs(speed);
    drive(-speed, -speed);
    Serial.print("BACKWARD, speed = ");
}

void turnLeft(int speed) {
    speed = abs(speed);
    drive(speed / 2, speed);
    Serial.print("TURNLEFT, speed = ");
}

void turnRight(int speed) {
    speed = abs(speed);
    drive(speed, speed/2);
    Serial.print("TURNRIGHT, speed = ");
}

void pivotLeft(int speed) {
    speed = abs(speed);
    drive(-speed, speed);
}

void pivotRight(int speed) {
    speed = abs(speed);
    drive(speed, -speed);
}

void motorCoast(MotorID motor) {
    motorSet(motor, 0);
}

void stopAllMotors() {
    for (int i = 0; i < 4; i++) {
        motorSet((MotorID)i, 0);
    }
}


void motorBrake(MotorID motor) {
    const int index = static_cast<int>(motor);
    const int channel1 = index * 2;
    const int channel2 = index * 2 + 1;

    ledcWrite(channel1, PWM_MAX);
    ledcWrite(channel2, PWM_MAX);
}
