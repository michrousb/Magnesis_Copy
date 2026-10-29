#pragma once

enum EncoderID {
    ENCODER_FRONT_LEFT = 0,
    ENCODER_FRONT_RIGHT,
    ENCODER_REAR_LEFT,
    ENCODER_REAR_RIGHT
};

void encoderInit();
long encoderGetCount(EncoderID encoder);
void encoderReset(EncoderID encoder);
void encoderResetAll();

float encoderGetRPM(EncoderID encoder);
