#include <Arduino.h>

#include "navigation.hpp"
#include "bfd.hpp"
#include "line_follower.hpp"
#include "intersection.hpp"
#include "motion.hpp"
#include "motor.h"
#include "maze.hpp"
#include "config.h"

static constexpr unsigned long NAV_START_EXIT_TIMEOUT_MS = 8000;
static constexpr unsigned long NAV_SCAN_TIMEOUT_MS = 8000;
static constexpr unsigned long NAV_RETURN_EDGE_TIMEOUT_MS = 30000;
static constexpr unsigned long NAV_START_FIND_TIMEOUT_MS = 5000;


static constexpr bool NAV_AUTO_RETURN = false;

enum class NavigationState {
    LEAVE_START,
    FOLLOW_LINE,
    SCAN_INTERSECTION,
    CHECK_FINISH,
    WAIT_FOR_DECISION,
    HANDLE_LOST_LINE,
    RETURN_FOLLOW_LINE,
    RETURN_SCAN_INTERSECTION,
    RETURN_FIND_START,
    RETURN_CONFIRM_START,
    FINISHED,
    MOTION_ERROR
};

static NavigationState state = NavigationState::LEAVE_START;
static bool allBlackThroughoutCheck = false;
static bool startBoxCleared = false;
static unsigned long stateStartedMs = 0;

static JunctionInfo currentJunction = {
    false, false, false, false, JunctionType::UNKNOWN
};

static long ticksBeforeIntersectionScan = 0;
static long intersectionScanTargetTicks = INTERSECTION_SCAN_TICKS;

static MazeReturnRoute returnRoute;
static int returnStepIndex = 0;
static bool returnSawNormalLine = false;

static long getAverageTravelTicks() {
    long left = motionGetLeftTicks();
    long right = motionGetRightTicks();
    if (left < 0) left = -left;
    if (right < 0) right = -right;
    return (left + right) / 2;
}

static bool rearSeesLine(const BFDReading &reading) {
    return reading.leftOuter || reading.leftInner ||
           reading.rightInner || reading.rightOuter;
}

// Penanda bahwa robot telah benar-benar meninggalkan junction/box asal.
// Hindari langsung mendeteksi box FINISH yang masih berada di bawah sensor.
static bool normalLine(const BFDReading &reading) {
    return rearSeesLine(reading) &&
           !bfdAllBlack(reading) && !intersectionCandidate(reading);
}

static void failNavigation(const char *message) {
    stopAllMotors();
    Serial.print("[NAV] ERROR: ");
    Serial.println(message);
    state = NavigationState::MOTION_ERROR;
}

static void printJunction(const JunctionInfo &junction) {
    Serial.print("Junction: ");
    Serial.println(junctionTypeToString(junction.type));
    Serial.print("L="); Serial.print(junction.left);
    Serial.print(" S="); Serial.print(junction.straight);
    Serial.print(" R="); Serial.print(junction.right);
    Serial.print(" B="); Serial.println(junction.back);
}

static void printMazeMove(MazeMove move) {
    Serial.print("Maze move: ");
    Serial.println(mazeMoveToString(move));
}

static bool pivotForMove(MazeMove move) {
    switch (move) {
        case MazeMove::LEFT: return turnLeft90();
        case MazeMove::STRAIGHT: return true;
        case MazeMove::RIGHT: return turnRight90();
        case MazeMove::BACK: return turnAround180();
        case MazeMove::NONE: return false;
    }
    return false;
}

static void executeMazeMove(MazeMove move) {
    stopAllMotors();
    if (move == MazeMove::NONE) {
        failNavigation("DFS tidak menghasilkan gerakan valid");
        return;
    }
    if (!pivotForMove(move)) {
        failNavigation("Pivot eksplorasi gagal");
        return;
    }
    motionResetProgress();
    lineFollowerReset();
    state = NavigationState::FOLLOW_LINE;
}

static void finishIntersectionScan(long scanTicks) {
    stopAllMotors();
    mazeRecordTravelDistance(ticksBeforeIntersectionScan + scanTicks);
    currentJunction = intersectionScanFinish(true);
    printJunction(currentJunction);
    state = NavigationState::WAIT_FOR_DECISION;
}

static void beginReturnStep() {
    if (returnStepIndex < 0 || returnStepIndex >= returnRoute.stepCount) {
        failNavigation("Indeks langkah pulang invalid");
        return;
    }
    const MazeReturnStep &step = returnRoute.steps[returnStepIndex];
    Serial.print("[RETURN] Node "); Serial.print(step.fromNode);
    Serial.print(" -> "); Serial.print(step.toNode);
    Serial.print(", turn="); Serial.print(mazeMoveToString(step.turn));
    Serial.print(", saved ticks="); Serial.println(step.distanceTicks);

    stopAllMotors();
    if (!pivotForMove(step.turn)) {
        failNavigation("Pivot pulang gagal");
        return;
    }
    motionResetProgress();
    lineFollowerReset();
    returnSawNormalLine = false;
    stateStartedMs = millis();
    state = NavigationState::RETURN_FOLLOW_LINE;
}

static void startReturnOrFinish() {
    if (!mazeHasSavedMap()) {
        failNavigation("Snapshot map FINISH gagal dibuat");
        return;
    }
    Serial.print("[NAV] Map saved. nodes=");
    Serial.print(mazeGetSavedNodeCount());
    Serial.print(", DFS path nodes=");
    Serial.println(mazeGetSavedPathCount());

    if (!NAV_AUTO_RETURN) {
        stopAllMotors();
        state = NavigationState::FINISHED;
        return;
    }
    if (!mazeBuildReturnRoute(returnRoute)) {
        failNavigation("Tidak bisa membangun rute DFS pulang");
        return;
    }
    returnStepIndex = 0;
    if (returnRoute.stepCount == 0) {
        stopAllMotors();
        state = NavigationState::FINISHED;
        return;
    }
    beginReturnStep();
}

static void finishReturnAtStart() {
    stopAllMotors();
    returnStepIndex = returnRoute.stepCount;
    state = NavigationState::FINISHED;
    Serial.println("[NAV] Tiba di box START. Robot berhenti.");
}

void navigationInit() {
    mazeInit(MazeHeading::NORTH);
    if (!mazeBeginFromStart()) {
        failNavigation("Gagal mendaftarkan box START sebagai node 0");
        return;
    }
    lineFollowerReset();
    intersectionScanReset();
    ticksBeforeIntersectionScan = 0;
    intersectionScanTargetTicks = INTERSECTION_SCAN_TICKS;
    allBlackThroughoutCheck = false;
    startBoxCleared = false;
    returnRoute = MazeReturnRoute{};
    returnStepIndex = 0;
    returnSawNormalLine = false;
    motionResetProgress();
    stateStartedMs = millis();
    state = NavigationState::LEAVE_START;
}

void navigationUpdate() {
    switch (state) {
        case NavigationState::LEAVE_START: {
            if (millis() - stateStartedMs > NAV_START_EXIT_TIMEOUT_MS) {
                failNavigation("Tidak berhasil meninggalkan box START");
                return;
            }
            BFDReading reading = bfdRead();
            if (normalLine(reading)) {
                startBoxCleared = true;
                lineFollowerReset();
                state = NavigationState::FOLLOW_LINE;
                return;
            }
            // Jangan menganggap box START sebagai FINISH/junction.
            forward(MOTION_FORWARD_SPEED);
            return;
        }

        case NavigationState::FOLLOW_LINE: {
            BFDReading reading = bfdRead();
            if (bfdAllBlack(reading)) {
                stopAllMotors();
                lineFollowerReset();
                intersectionScanReset();
                intersectionScanObserve(reading);
                ticksBeforeIntersectionScan = getAverageTravelTicks();
                allBlackThroughoutCheck = true;
                motionResetProgress();
                forward(MOTION_FORWARD_SPEED);
                stateStartedMs = millis();
                state = NavigationState::CHECK_FINISH;
                return;
            }
            if (intersectionCandidate(reading)) {
                stopAllMotors();
                lineFollowerReset();
                intersectionScanReset();
                intersectionScanObserve(reading);
                ticksBeforeIntersectionScan = getAverageTravelTicks();
                intersectionScanTargetTicks = INTERSECTION_SCAN_TICKS;
                motionResetProgress();
                forward(MOTION_FORWARD_SPEED);
                stateStartedMs = millis();
                state = NavigationState::SCAN_INTERSECTION;
                return;
            }
            LineFollowStatus status = lineFollowerUpdate(reading);
            if (status == LineFollowStatus::LOST) {
                stopAllMotors();
                state = NavigationState::HANDLE_LOST_LINE;
            }
            return;
        }

        case NavigationState::SCAN_INTERSECTION: {
            if (millis() - stateStartedMs > NAV_SCAN_TIMEOUT_MS) {
                failNavigation("Timeout scan intersection");
                return;
            }
            intersectionScanObserve(bfdRead());
            if (!motionTargetReached(intersectionScanTargetTicks)) return;
            finishIntersectionScan(getAverageTravelTicks());
            return;
        }

        case NavigationState::CHECK_FINISH: {
            if (millis() - stateStartedMs > NAV_SCAN_TIMEOUT_MS) {
                failNavigation("Timeout cek FINISH");
                return;
            }
            BFDReading reading = bfdRead();
            intersectionScanObserve(reading);
            if (!bfdAllBlack(reading)) allBlackThroughoutCheck = false;
            if (!motionTargetReached(FINISH_CHECK_TICKS)) return;
            stopAllMotors();
            long checkTicks = getAverageTravelTicks();
            // START sudah dilewati. FINISH boleh langsung setelah START,
            // tanpa harus ada persimpangan di antaranya.
            if (allBlackThroughoutCheck && startBoxCleared) {
                Serial.println("[NAV] FINISH detected!");
                mazeRecordTravelDistance(ticksBeforeIntersectionScan + checkTicks);
                mazeMarkGoalReached();
                startReturnOrFinish();
                return;
            }
            // Area hitam lebar tetapi bukan FINISH: lanjut scan junction.
            ticksBeforeIntersectionScan += checkTicks;
            long remaining = INTERSECTION_SCAN_TICKS - checkTicks;
            if (remaining <= 0) {
                finishIntersectionScan(0);
                return;
            }
            intersectionScanTargetTicks = remaining;
            motionResetProgress();
            forward(MOTION_FORWARD_SPEED);
            stateStartedMs = millis();
            state = NavigationState::SCAN_INTERSECTION;
            return;
        }

        case NavigationState::WAIT_FOR_DECISION: {
            stopAllMotors();
            if (mazeGoalReached() || mazeExplorationFinished()) {
                state = NavigationState::FINISHED;
                return;
            }
            MazeMove move = mazeChooseDirection(currentJunction);
            printMazeMove(move);
            if (move == MazeMove::NONE) {
                if (mazeExplorationFinished()) {
                    state = NavigationState::FINISHED;
                } else {
                    failNavigation("DFS tidak bisa memilih jalur");
                }
                return;
            }
            executeMazeMove(move);
            return;
        }

        case NavigationState::HANDLE_LOST_LINE: {
            stopAllMotors();
            // PERHATIAN: LOST belum tentu dead end secara fisik.
            // Pertahankan asumsi lama sampai tersedia sensor/konfirmasi buntu.
            mazeRecordTravelDistance(getAverageTravelTicks());
            MazeMove move = mazeHandleDeadEnd();
            Serial.println("[NAV] LOST dianggap dead end (perlu kalibrasi)");
            printMazeMove(move);
            if (move == MazeMove::NONE) {
                if (mazeExplorationFinished()) state = NavigationState::FINISHED;
                else failNavigation("Gagal backtrack dari LOST");
                return;
            }
            executeMazeMove(move);
            return;
        }

        case NavigationState::RETURN_FOLLOW_LINE: {
            if (millis() - stateStartedMs > NAV_RETURN_EDGE_TIMEOUT_MS) {
                failNavigation("Timeout mencari node berikutnya saat pulang");
                return;
            }
            if (returnStepIndex >= returnRoute.stepCount) {
                failNavigation("Rute pulang selesai tanpa deteksi START");
                return;
            }
            const MazeReturnStep &step = returnRoute.steps[returnStepIndex];
            // Distance hanya batas pengaman: posisi junction ditentukan sensor.
            long traveled = getAverageTravelTicks();
            long maxTicks = step.distanceTicks * 3L +
                            2L * INTERSECTION_SCAN_TICKS + FINISH_CHECK_TICKS + 100L;
            if (step.distanceTicks >= 0 && traveled > maxTicks) {
                failNavigation("Jarak pulang terlalu jauh tanpa menemukan node");
                return;
            }
            BFDReading reading = bfdRead();
            if (!returnSawNormalLine && normalLine(reading)) {
                returnSawNormalLine = true;
            }
            if (returnSawNormalLine &&
                (bfdAllBlack(reading) || intersectionCandidate(reading))) {
                stopAllMotors();
                Serial.print("[RETURN] Node candidate, measured ticks=");
                Serial.println(traveled);
                if (step.toNode == 0) {
                    // Root adalah box START, bukan junction biasa.
                    // Sensor rear bisa mendeteksi 3 hitam sebelum front hitam.
                    stateStartedMs = millis();
                    state = NavigationState::RETURN_FIND_START;
                    forward(MOTION_FORWARD_SPEED);
                    return;
                }
                intersectionScanReset();
                intersectionScanObserve(reading);
                motionResetProgress();
                forward(MOTION_FORWARD_SPEED);
                stateStartedMs = millis();
                state = NavigationState::RETURN_SCAN_INTERSECTION;
                return;
            }
            LineFollowStatus status = lineFollowerUpdate(reading);
            if (status == LineFollowStatus::LOST) {
                failNavigation("Garis hilang saat pulang");
            }
            return;
        }

        case NavigationState::RETURN_SCAN_INTERSECTION: {
            if (millis() - stateStartedMs > NAV_SCAN_TIMEOUT_MS) {
                failNavigation("Timeout scan junction saat pulang");
                return;
            }
            intersectionScanObserve(bfdRead());
            if (!motionTargetReached(INTERSECTION_SCAN_TICKS)) return;
            stopAllMotors();
            // Junction berikutnya tercapai; route sudah fixed (tidak DFS lagi).
            ++returnStepIndex;
            if (returnStepIndex >= returnRoute.stepCount) {
                failNavigation("Rute berakhir di junction, bukan box START");
                return;
            }
            beginReturnStep();
            return;
        }

        case NavigationState::RETURN_FIND_START: {
            if (millis() - stateStartedMs > NAV_START_FIND_TIMEOUT_MS) {
                failNavigation("Box START tidak menjadi full black");
                return;
            }
            BFDReading reading = bfdRead();
            if (bfdAllBlack(reading)) {
                motionResetProgress();
                forward(MOTION_FORWARD_SPEED);
                stateStartedMs = millis();
                state = NavigationState::RETURN_CONFIRM_START;
            } else {
                forward(MOTION_FORWARD_SPEED);
            }
            return;
        }

        case NavigationState::RETURN_CONFIRM_START: {
            if (millis() - stateStartedMs > NAV_SCAN_TIMEOUT_MS) {
                failNavigation("Timeout konfirmasi box START");
                return;
            }
            if (!bfdAllBlack(bfdRead())) {
                failNavigation("Marker START tidak full black sepanjang cek");
                return;
            }
            if (motionTargetReached(FINISH_CHECK_TICKS)) {
                finishReturnAtStart();
            }
            return;
        }

        case NavigationState::FINISHED:
        case NavigationState::MOTION_ERROR:
            stopAllMotors();
            return;
    }
}

bool navigationHasError() { return state == NavigationState::MOTION_ERROR; }
bool navigationIsReturning() {
    return state == NavigationState::RETURN_FOLLOW_LINE ||
           state == NavigationState::RETURN_SCAN_INTERSECTION ||
           state == NavigationState::RETURN_FIND_START ||
           state == NavigationState::RETURN_CONFIRM_START;
}
bool navigationIsFinished() { return state == NavigationState::FINISHED; }
