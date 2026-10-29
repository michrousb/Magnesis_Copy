#include <Arduino.h>

#include "navigation.hpp"
#include "bfd.hpp"
#include "line_follower.hpp"
#include "intersection.hpp"
#include "motion.hpp"
#include "motor.h"
#include "maze.hpp"
#include "config.h"


enum class NavigationState {
    FOLLOW_LINE,
    SCAN_INTERSECTION,
    WAIT_FOR_DECISION,
    HANDLE_LOST_LINE
};


static NavigationState state = NavigationState::FOLLOW_LINE;


// hasil junction terakhir
static JunctionInfo currentJunction = {
    false,
    false,
    false,
    false,
    JunctionType::UNKNOWN
};

// jarak dari node sebelumnya sampai awal intersection
static long ticksBeforeIntersectionScan = 0;


// ambil rata-rata encoder kiri dan kanan
static long getAverageTravelTicks() {

    long leftTicks = motionGetLeftTicks();

    long rightTicks = motionGetRightTicks();

    if (leftTicks < 0) {
        leftTicks = -leftTicks;
    }

    if (rightTicks < 0) {
        rightTicks = -rightTicks;
    }
    return (leftTicks +rightTicks) / 2;
}


// DEBUG

static void printJunction(
    const JunctionInfo &junction
) {

    Serial.print("Junction: ");

    Serial.println(
        junctionTypeToString(
            junction.type
        )
    );


    Serial.print("L=");
    Serial.print(junction.left);

    Serial.print(" S=");
    Serial.print(junction.straight);

    Serial.print(" R=");
    Serial.print(junction.right);

    Serial.print(" B=");
    Serial.println(junction.back);
}


static void printMazeMove(
    MazeMove move
) {

    Serial.print("Maze move: ");

    Serial.println(
        mazeMoveToString(move)
    );
}


// EXECUTE MAZE DECISION

static void executeMazeMove(
    MazeMove move
) {

    stopAllMotors();
    switch (move) {

        case MazeMove::LEFT:
            turnLeft90();
            break;

        case MazeMove::STRAIGHT:
            break;

        case MazeMove::RIGHT:
            turnRight90();
            break;

        case MazeMove::BACK:
            turnAround180();
            break;

        case MazeMove::NONE:
            return;
    }

    motionResetProgress();
    lineFollowerReset();


    state = NavigationState::FOLLOW_LINE;
}


void navigationInit() {

    state = NavigationState::FOLLOW_LINE;
    lineFollowerReset();
    intersectionScanReset();
    mazeInit();
    ticksBeforeIntersectionScan = 0;
    motionResetProgress();
}

void navigationUpdate() {

    switch (state) {
        // FOLLOW LINE
        case NavigationState::FOLLOW_LINE: {
            BFDReading reading =
                bfdRead();

            // 1. CEK INTERSECTION DULU
            if (intersectionCandidate(reading) ) {

                stopAllMotors();
                lineFollowerReset();
                intersectionScanReset();
                intersectionScanObserve(reading);

                ticksBeforeIntersectionScan = getAverageTravelTicks();
                motionResetProgress();

                forward(MOTION_FORWARD_SPEED);


                state =NavigationState::SCAN_INTERSECTION;

                return;
            }

            // 2. NORMAL LINE FOLLOWING

            LineFollowStatus followStatus = lineFollowerUpdate(reading);
            if (followStatus ==LineFollowStatus::TRACKING) {
                return;
            }

            if (followStatus ==LineFollowStatus::RECOVERING
            ) {
                return;
            }

            // 3. SHORT RECOVERY GAGAL
           
            if (followStatus ==LineFollowStatus::LOST) {
                stopAllMotors();
                state = NavigationState::HANDLE_LOST_LINE;
                return;
            }
            break;
        }

        // SCAN INTERSECTION


        case NavigationState::SCAN_INTERSECTION: {

            BFDReading reading = bfdRead();
            intersectionScanObserve(reading);

  
            if (!motionTargetReached(INTERSECTION_SCAN_TICKS)
            ) {
                  return;
            }
            stopAllMotors();


            long scanTicks = getAverageTravelTicks();


            long totalEdgeTicks = ticksBeforeIntersectionScan + scanTicks;
            mazeRecordTravelDistance(totalEdgeTicks);

            currentJunction =intersectionScanFinish(true);

            printJunction(currentJunction);


            state = NavigationState::WAIT_FOR_DECISION;


            return;
        }


        // DFS DECISION
        
        case NavigationState::WAIT_FOR_DECISION: {
            stopAllMotors();
            if (mazeExplorationFinished()) {
                return;
            }
            if (mazeGoalReached()) {
                return;
            }
            MazeMove move =mazeChooseDirection(currentJunction);
            printMazeMove(
                move
            );
            if (move ==MazeMove::NONE
            ) {
                stopAllMotors();
                return;
            }

            executeMazeMove(move);
            return;
        }


        // DEAD END
       
        case NavigationState::HANDLE_LOST_LINE: {
            stopAllMotors();
            long deadEndDistance = getAverageTravelTicks();
            mazeRecordTravelDistance( deadEndDistance);
            MazeMove move = mazeHandleDeadEnd();

            Serial.println("Ada Dead end");
            printMazeMove(move);

            if (move ==MazeMove::NONE
            ) {
                stopAllMotors();
                return;
            }
            executeMazeMove(move);
            return;
        }
    }
}