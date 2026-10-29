#include <Arduino.h>
#include "encoder.h"
#include "config.h"

struct EncoderConfig {
    uint8_t pinA;
    uint8_t pinB;
};

static const EncoderConfig encoder[4] = { //encoder[FRONT_LEFT].pinA = 34 (pin)
    {FL_ENC_A, FL_ENC_B}, //index 0 : front left
    {FR_ENC_A, FR_ENC_B}, // index 1 : front right
    {RL_ENC_A, RL_ENC_B}, // index 2  :rear left
    {RR_ENC_A, RR_ENC_B}, // Index 3 : rear Right
};

volatile long encoderCount[4] = {0, 0, 0, 0};

void IRAM_ATTR encoderFLISR(){ //ISR = Interrupt Service Routine. IRAM_ATTR tempat naro fungsi biar cepet diakses pas interrupt
    if(digitalRead(FL_ENC_B)){ // interrupt terjadi saat A berubah low ke high. B menentukan arah putar
        encoderCount[ENCODER_FRONT_LEFT]++;
    }
    else{
        encoderCount[ENCODER_FRONT_LEFT]--;
    }
}

void IRAM_ATTR encoderFRISR(){ 
    if(digitalRead(FR_ENC_B)){ 
        encoderCount[ENCODER_FRONT_RIGHT]++;
    }
    else{
        encoderCount[ENCODER_FRONT_RIGHT]--;
    }
}

void IRAM_ATTR encoderRLISR(){ 
    if(digitalRead(RL_ENC_B)){ 
        encoderCount[ENCODER_REAR_LEFT]++;
    }
    else{
        encoderCount[ENCODER_REAR_LEFT]--;
    }
}

void IRAM_ATTR encoderRRISR(){ 
    if(digitalRead(RR_ENC_B)){ 
        encoderCount[ENCODER_REAR_RIGHT]++;
    }
    else{
        encoderCount[ENCODER_REAR_RIGHT]--;
    }
}

void encoderInit(){
    for (int i =0; i< 4; i++){
        pinMode(encoder[i].pinA, INPUT); // pin A input (encoder -> ESP32)
        pinMode(encoder[i].pinB, INPUT);

    }
    attachInterrupt( //setiap kali pin FL_ENC_A RISing (0 OTW 1), jalanin encoderFLISR (cek keadaan B)
        digitalPinToInterrupt(FL_ENC_A),
        encoderFLISR,
        RISING
    );
    attachInterrupt( 
        digitalPinToInterrupt(FR_ENC_A),
        encoderFRISR,
        RISING
    );
    attachInterrupt( 
        digitalPinToInterrupt(RL_ENC_A),
        encoderRLISR,
        RISING
    );
    attachInterrupt( 
        digitalPinToInterrupt(RR_ENC_A),
        encoderRRISR,
        RISING
    );

}

long encoderGetCount(EncoderID id){
    noInterrupts(); //matiin interrupt biar ga race condition
    long count = encoderCount[id];
    interrupts();
    return count; // count-> seberapa banyak rotasi/pergerakan yg terjadi
}

void encoderReset(EncoderID encoder){
    noInterrupts();
    encoderCount[encoder] = 0;
    interrupts();
}

void encoderResetAll(){
    noInterrupts();
    for (int i =0; i<4; i++){
        encoderCount[i] = 0;
    }
    interrupts();
}
