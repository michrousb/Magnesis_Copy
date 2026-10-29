#pragma once
/*ada N E S W (notrh ease south west) buat ngasih tau ini pergerakan absolutnya
turnLeft/Right/Around() : arah absolut buat helper logika arah. jd klo turnAround (NORTH) = SOUTH.
- bedanya sm motion command yg turnLeft90 itu cmn muter fisik. klo ini lebih high level
kita pakai graph bukan cells karena kita ga fixed-cell-distance (maju satu cell = maju satu edge)
tiap node :
 - uint8_t exits -> ada jalan di arah x
 - uint8_t known -> arah udah pernah di scan
 - uint7_t tried -> dari node ini, exit mana yang udh pernah dicoba? cth 
                    exits = 1011
                    tried = 0011
                    arti:   NORTH ada  → sudah dicoba
                            EAST ada   → sudah dicoba
                            SOUTH tidak ada
                            WEST ada   → BELUM dicoba
 - bool deadEnd -> node buntu
 - neighbor[N/E/S/W] buat tahu nesw nya ada node berapa. klo belum tahu -1
 - edgeDistance[4] ngasih tau distance ke node lain berapa ticks. contoh nodes[A].edgeDistance[EAST]  = 800;
 
 bikin map yg isinua node biar setelah sekali mapping udh punya map.
 bikin juga fungsi mirroring, misal A ke B 800 ticks maka B ke A jg 800 ticks
 bikin juga hasExit (ada jalur), isKnown(udah pernah diperiksa blm), isExit
 Open itu isKnown (udh pernah diperiksa) dan bisa dilewati
 Bikin mazeGoalReached() janlup
 exitCount() buat ngasih tau ada berapa exit
 junctionToAbsoluteMask buat ngasih tau heading robot absolut (cth: kiri nya itu south)
 nodeCount itu jmlh node (debug)*/
#include <stdint.h>
#include "intersection.hpp"


enum class MazeHeading : uint8_t {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};


enum class MazeMove : uint8_t {
    LEFT,
    STRAIGHT,
    RIGHT,
    BACK,
    NONE
}; // yg dikirim dri keputusan DFS ke navigation.cpp


void mazeInit(MazeHeading startHeading = MazeHeading::NORTH);

//DFS
MazeMove mazeChooseDirection(const JunctionInfo &junction);
MazeMove mazeHandleDeadEnd();
void mazeRecordTravelDistance(long distanceTicks);

//status
bool mazeExplorationFinished();
bool mazeGoalReached();
int mazeGetNodeCount();

//debug
const char* mazeMoveToString(MazeMove move);