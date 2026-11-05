#include <Arduino.h>

#include "intersection.hpp"

/*bit:     4    3    2    1    0
sensor:  F   LO   LI   RI   RO
*/

// BIT POSITION

static constexpr uint8_t BIT_FRONT =
    (1u << 4); //10000 (unsigned)

static constexpr uint8_t BIT_LEFT_OUTER =
    (1u << 3);

static constexpr uint8_t BIT_LEFT_INNER =
    (1u << 2);

static constexpr uint8_t BIT_RIGHT_INNER =
    (1u << 1);

static constexpr uint8_t BIT_RIGHT_OUTER =
    (1u << 0);


// BIT MASK (-01111)
static constexpr uint8_t MASK_REAR = BIT_LEFT_OUTER | BIT_LEFT_INNER | BIT_RIGHT_INNER | BIT_RIGHT_OUTER;

// SCAN MEMORY

static bool sawLeft = false;
static bool sawRight = false;
static bool hasReading = false;
static uint8_t lastBits = 0; //kondisi sensor paling akhir

// UBAH BFDReading MENJADI BITMASK
static uint8_t sensorBits(const BFDReading &reading) {

    uint8_t bits = 0;

    if (reading.front) {
        bits |= BIT_FRONT;
    }
    if (reading.leftOuter) {
        bits |= BIT_LEFT_OUTER;
    }
    if (reading.leftInner) {
        bits |= BIT_LEFT_INNER;
    }
    if (reading.rightInner) {
        bits |= BIT_RIGHT_INNER;
    }
    if (reading.rightOuter) {
        bits |= BIT_RIGHT_OUTER;
    }
    return bits;
}


static int countSetBits(uint8_t bits) {
    int count = 0;
    while (bits != 0) {
        count += bits & 1u; //ngambil LSB
        bits >>= 1;
    }
    return count;
}


static JunctionType classifyJunction(bool left,bool straight,bool right) {

    if (left &&straight &&right
    ) {
        return JunctionType::CROSS;
    }


    if (left &&!straight &&right
    ) {
        return JunctionType::T_JUNCTION;
    }


    if (left &&straight &&!right
    ) {
        return JunctionType::LEFT_STRAIGHT;
    }

    if (!left &&straight &&right
    ) {
        return JunctionType::RIGHT_STRAIGHT;
    }

    if (left &&!straight &&!right
    ) {
        return JunctionType::LEFT_ONLY;
    }

    if (!left &&!straight &&right
    ) {
        return JunctionType::RIGHT_ONLY;
    }


    if (!left &&straight &&!right
    ) {
        return JunctionType::STRAIGHT_ONLY;
    }


    if (!left &&!straight &&!right
    ) {
        return JunctionType::DEAD_END;
    }
    return JunctionType::UNKNOWN;
}

// CEK CANDIDATE INTERSECTION
bool intersectionCandidate(const BFDReading &reading) {

    uint8_t bits =sensorBits(reading);
    uint8_t rearBits = bits & MASK_REAR;

    int rearActiveCount = countSetBits(rearBits);
    return rearActiveCount >= 3;
}


// ======================================================
// RESET SCAN
// ======================================================

void intersectionScanReset() {
    sawLeft = false;
    sawRight = false;
    hasReading = false;
    lastBits = 0;
}


// ======================================================
// OBSERVE SELAMA SCAN
// ======================================================

void intersectionScanObserve(
    const BFDReading &reading
) {

    uint8_t bits =sensorBits(reading);


    // Kalau sensor paling kiri melihat garis,
    // simpan evidence bahwa ada jalur kiri.
    if (bits & BIT_LEFT_OUTER) {
        sawLeft = true;
    }


    // Kalau sensor paling kanan melihat garis,
    // simpan evidence bahwa ada jalur kanan.
    if (bits &BIT_RIGHT_OUTER) {
        sawRight = true;
    }
    // Simpan snapshot terakhir.
    lastBits = bits;
    hasReading = true;
}


// ======================================================
// SELESAIKAN SCAN
// ======================================================

JunctionInfo intersectionScanFinish(bool hasBack) {

    JunctionInfo info;


    // Kalau observe() belum pernah dipanggil,
    // kita tidak punya cukup data.
    if (!hasReading) {
        info.left = false;
        info.straight = false;
        info.right = false;
        info.back = hasBack;
        info.type =JunctionType::UNKNOWN;
        return info;
    }

    info.left =sawLeft;
    info.right =sawRight;
    // Sensor FRONT pada posisi akhir scan
    // dipakai untuk mengecek apakah jalur
    // masih berlanjut lurus.
    info.straight = (lastBits & BIT_FRONT) != 0;
    info.back =hasBack;

    info.type = classifyJunction(info.left,info.straight,info.right);
    return info;
}


// ======================================================
// DEBUG STRING
// ======================================================

const char* junctionTypeToString(
    JunctionType type
) {

    switch (type) {

        case JunctionType::NORMAL:
            return "NORMAL";

        case JunctionType::LEFT_ONLY:
            return "LEFT_ONLY";

        case JunctionType::RIGHT_ONLY:
            return "RIGHT_ONLY";

        case JunctionType::STRAIGHT_ONLY:
            return "STRAIGHT_ONLY";

        case JunctionType::LEFT_STRAIGHT:
            return "LEFT_STRAIGHT";

        case JunctionType::RIGHT_STRAIGHT:
            return "RIGHT_STRAIGHT";

        case JunctionType::T_JUNCTION:
            return "T_JUNCTION";

        case JunctionType::CROSS:
            return "CROSS";

        case JunctionType::DEAD_END:
            return "DEAD_END";

        default:
            return "UNKNOWN";
    }
}