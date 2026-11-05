#pragma once
// == MOTOR ==
// Front Left
#define FL_IN1 13
#define FL_IN2 12

// Front Right
#define FR_IN1 27
#define FR_IN2 26

// Rear Left
#define RL_IN1 25
#define RL_IN2 33

// Rear Right
#define RR_IN1 32
#define RR_IN2 14
// PWM CONFIG


#define PWM_FREQ       20000   // 20 kHz, di atas audible range
#define PWM_RESOLUTION 8 // 0-255
#define PWM_MAX        255


// MOTOR DIRECTION
// Karena motor kiri/kanan dipasang mirror,
// arah listriknya mungkin terbalik.
//
// false = normal
// true  = balik arah (coba cek lgi siapa tau kebalik)

#define FL_INVERTED false
#define FR_INVERTED true
#define RL_INVERTED false
#define RR_INVERTED true

// == ENCODER ==

// Encoder Front Left
#define FL_ENC_A 34
#define FL_ENC_B 35

// Encoder Front Right
#define FR_ENC_A 36 //kyna klo pake bluetooth ini hrs direvisi
#define FR_ENC_B 39

// Encoder Rear Left
#define RL_ENC_A 18
#define RL_ENC_B 19

// Encoder Rear Right
#define RR_ENC_A 21
#define RR_ENC_B 22

// tentatif, nanti cek dengan cara putar roda 1 revolusi berapa encoderCountnya
#define ENCODER_PPR 360

// == BFD-1000 ==

#define BFD_FRONT 4
#define BFD_LEFT_OUTER 5
#define BFD_LEFT_INNER 16
#define BFD_RIGHT_OUTER 17
#define BFD_RIGHT_INNER 23


// Tentatif.
//
// Kalau sensor menghasilkan LOW saat melihat garis,
// gunakan LOW.
//
// Kalau ternyata HIGH saat melihat garis,
// ganti menjadi HIGH.
#define BFD_ACTIVE_LEVEL LOW

// == LINE FOLLOWER ==

#define LF_BASE_SPEED       120
#define LF_CURVE_SPEED      90
#define LF_MIN_SPEED        60

#define LF_KP               40.0f
#define LF_KD               20.0f

#define LF_FILTER_ALPHA     0.20f
#define LF_SPEED_DROP       10.0f

#define LF_MAX_CORRECTION   100

#define LF_RECOVERY_SLOW_SPEED  50
#define LF_RECOVERY_FAST_SPEED  90
#define LF_SHORT_RECOVERY_MS    150

// == Motion == INI SEMUA TENTATIF JUGA
#define MOTION_FORWARD_SPEED 90
#define MOTION_TURN_SPEED 100
#define TURN_90_TICKS 180 //tentatif, ini jmlh encoderCount untuk pivot 90 derajat
#define TURN_180_TICKS 360 //tentatif
#define INTERSECTION_SCAN_TICKS 60 //tentatif, case klo maju kecil di junction
#define MOTION_TIMEOUT_MS 2000 //safety timeout, klo encoder gagal bacca, motor ga muter terus

// CONVEYOR
constexpr int CONVEYOR_IN1 = 4;
constexpr int CONVEYOR_IN2 = 5;
#define CONVEYOR_PWM_CHANNEL 8
//PWM
constexpr int CONVEYOR_PWM_FREQ = 20000;
constexpr int CONVEYOR_PWM_RESOLUTION = 8;
constexpr int CONVERYOR_PWM_MAX = (1<<CONVEYOR_PWM_RESOLUTION) - 1;
constexpr int CONVEYOR_DEFAULT_SPEED = 150;

#define FINISH_CHECK_TICKS 50
