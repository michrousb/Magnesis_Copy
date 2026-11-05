/*
ada N E S W (notrh east south west) buat ngasih tau ini pergerakan absolutnya
turnLeft/Right/Around() : arah absolut buat helper logika arah. jd klo turnAround (NORTH) = SOUTH.
- bedanya sm motion command yg turnLeft90 itu cmn muter fisik. klo ini lebih high level
kita pakai graph bukan cells karena kita ga fixed-cell-distance (maju satu cell = maju satu edge)
tiap node :
 - uint8_t exits -> ada jalan di arah x yang UDH DI SCAN
 - uint8_t known -> arah udah pernah di scan [GA MUNGKIN knwon = 0 exit = 1]
 - uint7_t tried -> dari node ini, exit mana yang udh pernah dicoba? cth 
                    exits = 1011
                    tried = 0011
                    arti:   NORTH ada  → sudah dicoba
                            EAST ada   → sudah dicoba
                            SOUTH tidak ada
                            WEST ada   → BELUM dicoba
 - bool deadEnd -> node buntu
 - neighbor[N/E/S/W] buat tahu nesw nya ada node apa. klo belum tahu -1
 - edgeDistance[4] ngasih tau distance ke node lain berapa ticks. contoh nodes[A].edgeDistance[EAST]  = 800;
 
 bikin map yg isinua node biar setelah sekali mapping udh punya map.
 bikin juga fungsi mirroring, misal A ke B 800 ticks maka B ke A jg 800 ticks
 bikin juga hasExit (ada jalur), isKnown(udah pernah diperiksa blm), isExit
 Open itu isKnown (udh pernah diperiksa) dan bisa dilewati
 Bikin mazeGoalReached() janlup
 exitCount() buat ngasih tau ada berapa exit
 junctionToAbsoluteMask buat ngasih tau heading robot absolut (cth: kiri nya itu south)
 nodeCount itu jmlh node (debug)
 
*/
#include "maze.hpp"


static constexpr int MAX_MAZE_NODES = 64; //tentatif, ini buta nyimpen semua node pada map
static constexpr uint8_t ALL_DIRECTIONS = 0b00001111;

struct MazeNode{
    uint8_t exits = 0;
    uint8_t known = 0;
    uint8_t tried = 0;
    int neighbor[4] = {-1, -1, -1, -1};
    long edgeDistance[4] = {-1, -1, -1, -1};
    bool deadEnd = false;
};

static MazeNode nodes[MAX_MAZE_NODES];

//saved map
static MazeNode savedNodes[MAX_MAZE_NODES];
static int savedNodeCount = 0;
static int savedGoalNode = -1;
static MazeHeading savedGoalHeading = MazeHeading::NORTH;
static int savedPath[MAX_MAZE_NODES];
static int savedPathCount = 0;
static bool mapSaved = false;

static int nodeCount =0;
static int currentNode = -1;
static int dfsStack[MAX_MAZE_NODES];
static int stackTop = 0;
static int goalNode = -1;
static MazeHeading heading = MazeHeading :: NORTH;

//status
static bool explorationFinished = false;
static bool goalReached = false;

//distance
static long lastTravelDistanceTicks = -1;

//pending klo robot blm nyampe ke node target
struct PendingMove{
    bool active = false;
    int fromNode = -1;
    int targetNode = -1;
    MazeHeading direction = MazeHeading :: NORTH;
    bool backtracking = false; //explore/backtrack
};
static PendingMove pending;

//helpers
static bool validNode(int node){
    return node >= 0 && node < nodeCount;
}
static uint8_t headingBit(MazeHeading direction){ //ngubah enum n/e/s/w jadi bit
    return static_cast<uint8_t>(1u << static_cast<uint8_t>(direction));
}
//misal awalnya south trs left brti west
static MazeHeading turnLeftAbsolute(MazeHeading current){
    int value = static_cast<int>(current);
    value = (value+3) % 4;
    return static_cast<MazeHeading>(value);
}
static MazeHeading turnRightAbsolute(MazeHeading current){
    int value = static_cast<int>(current);
    value = (value+1) % 4;
    return static_cast<MazeHeading>(value);
}
static MazeHeading turnAroundAbsolute(MazeHeading current){
    int value = static_cast<int>(current);
    value = (value+2) % 4;
    return static_cast<MazeHeading>(value);
}

//map query
static bool isKnown(int node, MazeHeading direction){
    if(!validNode(node)){
        return false;
    }
    uint8_t bit = headingBit(direction);
    return (nodes[node].known & bit) != 0;
}
static bool hasExit(int node, MazeHeading direction){
    if(!validNode(node)){
        return false;
    }
    uint8_t bit = headingBit(direction);
    return(nodes[node].exits&bit) != 0;

}
static bool isOpen(int node, MazeHeading direction){
    return isKnown(node, direction) && hasExit(node, direction);
}

//ngitung berapa bnyk 1 pada bit
static int countSetBits(uint8_t bits){
    int count = 0;
    while(bits!=0){
        count += bits & 1u;
        bits >>= 1;
    }
    return count;
}

static int exitCount(int node){
    if(!validNode(node)){
        return 0;
    }
    return countSetBits(nodes[node].exits);
}

static int createNode(){
    if(nodeCount >= MAX_MAZE_NODES){
        return -1;
    }
    int id = nodeCount;
    nodes[id] = MazeNode{};
    nodeCount++;
    return id;
}

static void connectNodes(int nodeA, MazeHeading directionFromA, int nodeB, long distanceTicks){
    if(!validNode(nodeA) || !validNode(nodeB)){
        return ;
    }
    MazeHeading directionFromB = turnAroundAbsolute(directionFromA);
    int indexA = static_cast<int>(directionFromA);
    int indexB = static_cast<int>(directionFromB);
    //a ke b
    nodes[nodeA].neighbor[indexA] = nodeB;
    nodes[nodeA].known |= headingBit(directionFromA);
    nodes[nodeA].exits |= headingBit(directionFromA);
    nodes[nodeA].tried |= headingBit(directionFromA);
    //b ke a
    nodes[nodeB].neighbor[indexB] = nodeA;
    nodes[nodeB].known |= headingBit(directionFromB);
    nodes[nodeB].exits |= headingBit(directionFromB);
    nodes[nodeB].tried |= headingBit(directionFromB);

    if(distanceTicks >= 0){
        nodes[nodeA].edgeDistance[indexA]=distanceTicks;
        nodes[nodeB].edgeDistance[indexB] = distanceTicks;
    }

}

static uint8_t junctionToAbsoluteMask(const JunctionInfo &junction, MazeHeading currentHeading){
    uint8_t mask = 0;
    if(junction.left){
        mask |= headingBit(turnLeftAbsolute(currentHeading));
    }
    if(junction.straight){
        mask |= headingBit(currentHeading);
    }
    if(junction.right){
        mask |= headingBit(turnRightAbsolute(currentHeading));
    }
    if(junction.back){
        mask |= headingBit(turnAroundAbsolute(currentHeading));
    }
    return mask;
}

static void updateNodeFromJunction(int node, const JunctionInfo &junction){
    if(!validNode(node)){
        return;
    }
    uint8_t available = junctionToAbsoluteMask(junction, heading);
    nodes[node].known |= ALL_DIRECTIONS;
    nodes[node].exits |= available; //BNER GaKSI?? exit = 0 -> udh diperiksa dan tidk ada jalan
}

static MazeMove absoluteToRelative(MazeHeading currentHeading, MazeHeading targetHeading){
    int difference = (static_cast<int>(targetHeading) - static_cast<int>(currentHeading) + 4) %4;
    if (difference == 0){
        return MazeMove :: STRAIGHT;
    }
    if(difference == 1){
        return MazeMove :: RIGHT;
    }
    if(difference == 2){
        return MazeMove :: BACK;
    }
    return MazeMove :: LEFT;
}

//dipanggil pas robot sudah tiba (status awal pending)
static bool completePendingArrival() {
    // belum ada pending move
    if (!pending.active) {
        if (currentNode == -1) {
            currentNode = createNode();
            if (currentNode == -1) {
                explorationFinished = true;
                return false;
            }
            dfsStack[0] = currentNode;
            stackTop = 1;
        }
        return true;
    }

    // sampai node yang SUDAH dikenal
    if (pending.targetNode >= 0) {
        int target = pending.targetNode;
        connectNodes(
            pending.fromNode,
            pending.direction,
            target,
            lastTravelDistanceTicks
        );
        currentNode = target;
        if (pending.backtracking && stackTop > 1) {
            stackTop--;
        }
        pending = PendingMove{};
        lastTravelDistanceTicks = -1;
        return true;
    }

    //sampai node BARU
    int child = createNode();
    if (child == -1) {
        explorationFinished = true;
        pending = PendingMove{};
        return false;
    }
    connectNodes(pending.fromNode,pending.direction,child,lastTravelDistanceTicks);
    currentNode = child;
    if (stackTop < MAX_MAZE_NODES) {
        dfsStack[stackTop] = child;
        stackTop++;
    }
    pending = PendingMove{};
    lastTravelDistanceTicks = -1;
    return true;
}

static bool pickUnexploredExit(MazeHeading &selected){
    if(!validNode(currentNode)){
        return false;
    }
    //prioritas straight, left, right, back
    MazeHeading order[4] = {heading, turnLeftAbsolute(heading), turnRightAbsolute(heading), turnAroundAbsolute(heading)};
    for(int i =0; i<4; i++){
        MazeHeading direction = order[i];
        uint8_t bit = headingBit(direction);
        if(!isOpen(currentNode, direction)){
            continue;
        }
        if(nodes[currentNode].tried & bit){
            continue;
        } 
        selected = direction;
        return true;
    }
    return false;
}

static bool findParentDirection(MazeHeading &direction){ //cari parent buat backtracking
    if(stackTop <= 1 || !validNode(currentNode)){
        return false;
    }
    int parentNode = dfsStack[stackTop - 2];
    for(int i =0; i<4; i++){
        if(nodes[currentNode].neighbor[i] == parentNode){
            direction = static_cast<MazeHeading>(i);
            return true;
        }  
    }
    return false;
}

//schedule normal dfs move
static MazeMove scheduleExplore(MazeHeading direction){
    if(!validNode(currentNode)){
        return MazeMove :: NONE;
    }
    MazeMove move = absoluteToRelative(heading, direction);
    int index = static_cast<int>(direction); 
    //exit udh dipilih dfs
    nodes[currentNode].tried |= headingBit(direction);
    pending.active = true;
    pending.fromNode = currentNode;
    pending.targetNode = nodes[currentNode].neighbor[index];
    pending.direction = direction;
    pending.backtracking = false;
    heading = direction;
    return move;
}


static MazeMove scheduleBacktrack() {
    MazeHeading parentDirection;
    if (!findParentDirection(parentDirection)) {
        // Tidak ada parent lagi ->  kita sudah kembali ke root -> semua branch selesai.
        explorationFinished =true;
        return MazeMove::NONE;
    }

    MazeMove move = absoluteToRelative(heading, parentDirection);
    int parentNode = dfsStack[stackTop - 2];
    nodes[currentNode].tried |= headingBit(parentDirection);

    pending.active =true;
    pending.fromNode =currentNode;
    pending.targetNode =parentNode;
    pending.direction = parentDirection;
    pending.backtracking = true;

    heading =parentDirection;
    return move;
}

void mazeInit(MazeHeading startHeading){
    for(int i = 0; i<MAX_MAZE_NODES; i++){
        nodes[i] = MazeNode{};
        dfsStack[i] = -1;
    }
    nodeCount = 0;
    currentNode = -1;
    stackTop = 0;
    goalNode = -1;
    heading = startHeading;
    explorationFinished = false;
    goalReached = false;
    lastTravelDistanceTicks = -1;
    pending = PendingMove{};
    savedNodeCount = 0;
    savedGoalNode = -1;
    savedGoalHeading = startHeading;
    savedPathCount = 0;
    for (int i = 0; i < MAX_MAZE_NODES; ++i) savedPath[i] = -1;
    mapSaved = false;
}


void mazeRecordTravelDistance(long distanceTicks){
    if(distanceTicks < 0){
        lastTravelDistanceTicks = -1;
        return;
    }
    lastTravelDistanceTicks = distanceTicks;
}

MazeMove mazeChooseDirection(const JunctionInfo &junction){
    if(goalNode) return MazeMove::NONE;
    if(!completePendingArrival()){
        return MazeMove::NONE;
    }
    updateNodeFromJunction(currentNode, junction);
    //untuk node pertama,tandai jalur back udah pernah dilewatin
    if(currentNode == 0 && stackTop == 1){
        MazeHeading back = turnAroundAbsolute(heading);
        nodes[currentNode].tried |= headingBit(back);
    }
    //find unexplored
    MazeHeading selected;
    if(pickUnexploredExit(selected)){
        return scheduleExplore(selected);
    }
    //gaada unexploredexit
    if(exitCount(currentNode) <= 1){
        nodes[currentNode].deadEnd = true;
    }
    return scheduleBacktrack();
}

//dead end
MazeMove mazeHandleDeadEnd(){
    if(goalNode) return MazeMove::NONE;
    if(!completePendingArrival()){
        return MazeMove::NONE;
    }
    if(!validNode(currentNode)){
        return MazeMove::NONE;
    }
    MazeNode &node = nodes[currentNode];
    node.deadEnd = true;
    //dead end udh diperiksa penuh
    node.known |= ALL_DIRECTIONS;
    //satu"nya exit cmn backtrack
    MazeHeading back = turnAroundAbsolute(heading);
    node.exits |= headingBit(back);
    node.tried |= headingBit(back);
    return scheduleBacktrack();
}

void mazeMarkGoalReached() {
    if(goalReached) return;
    if (!completePendingArrival()) {
        return;
    }
    goalNode = currentNode;
    goalReached = true;

    savedNodeCount = nodeCount;
    savedGoalNode = goalNode;
    savedGoalHeading = heading;
    for (int i = 0; i < savedNodeCount; ++i) {
        savedNodes[i] = nodes[i];
    }
    
    if (stackTop <= 0 || stackTop > MAX_MAZE_NODES ||
        dfsStack[0] != 0 || dfsStack[stackTop - 1] != goalNode) {
        savedPathCount = 0;
        mapSaved = false;
        return;
    }
    savedPathCount = stackTop;
    for (int i = 0; i < savedPathCount; ++i) {
        savedPath[i] = dfsStack[i];
    }
    mapSaved = true;
}


bool mazeHasSavedMap() { return mapSaved; }

int mazeGetSavedNodeCount() { return mapSaved ? savedNodeCount : 0; }

int mazeGetSavedGoalNode() { return mapSaved ? savedGoalNode : -1; }

int mazeGetSavedNeighbor(int node, MazeHeading direction) {
    int d = static_cast<int>(direction);
    if (!mapSaved || node < 0 || node >= savedNodeCount || d < 0 || d >= 4) return -1;
    return savedNodes[node].neighbor[d];
}

long mazeGetSavedEdgeDistance(int node, MazeHeading direction) {
    int d = static_cast<int>(direction);
    if (!mapSaved || node < 0 || node >= savedNodeCount || d < 0 || d >= 4) return -1;
    return savedNodes[node].edgeDistance[d];
}

uint8_t mazeGetSavedExits(int node) {
    if (!mapSaved || node < 0 || node >= savedNodeCount) return 0;
    return savedNodes[node].exits;
}

int mazeGetSavedPathCount() {
    return mapSaved ? savedPathCount : 0;
}

int mazeGetSavedPathNode(int index) {
    if (!mapSaved || index < 0 || index >= savedPathCount) return -1;
    return savedPath[index];
}


bool mazeGoalReached() {
    return goalReached;
}

bool mazeExplorationFinished() {
    return explorationFinished;
}

int mazeGetNodeCount() {
    return nodeCount;
}

//debug
const char* mazeMoveToString(
    MazeMove move
) {

    switch (move) {

        case MazeMove::LEFT:
            return "LEFT";

        case MazeMove::STRAIGHT:
            return "STRAIGHT";

        case MazeMove::RIGHT:
            return "RIGHT";

        case MazeMove::BACK:
            return "BACK";

        default:
            return "NONE";
    }
}


const char* mazeHeadingToString(
    MazeHeading current
) {

    switch (current) {

        case MazeHeading::NORTH:
            return "NORTH";

        case MazeHeading::EAST:
            return "EAST";

        case MazeHeading::SOUTH:
            return "SOUTH";

        case MazeHeading::WEST:
            return "WEST";

        default:
            return "UNKNOWN";
    }
}

bool mazeBeginFromStart() {
    if (nodeCount != 0 || currentNode != -1 || pending.active || goalReached) {
        return false;
    }
    int start = createNode();
    if (start < 0) return false;
    currentNode = start; // START fisik = node 0
    dfsStack[0] = start;
    stackTop = 1;

    // Dari START, robot bergerak lurus mengikuti heading awal.
    nodes[start].known |= headingBit(heading);
    nodes[start].exits |= headingBit(heading);
    nodes[start].tried |= headingBit(heading);

    pending.active = true;
    pending.fromNode = start;
    pending.targetNode = -1; // persimpangan berikutnya belum diketahui
    pending.direction = heading;
    pending.backtracking = false;
    return true;
}

MazeHeading mazeGetSavedGoalHeading() {
    return savedGoalHeading;
}

// Rute pulang = jalur DFS aktif yang disimpan saat FINISH, dibaca TERBALIK.
// Tidak memakai BFS, queue, atau eksplorasi tambahan. Hanya memakai snapshot RAM.
// Memvalidasi setiap koneksi dan jarak dua arah sebelum menerima rute.
bool mazeBuildReturnRoute(MazeReturnRoute &route) {
    route = MazeReturnRoute{};
    if (!mapSaved || savedNodeCount <= 0 || savedNodeCount > MAX_MAZE_NODES ||
        savedPathCount <= 0 || savedPathCount > savedNodeCount ||
        savedGoalNode < 0 || savedGoalNode >= savedNodeCount ||
        savedPath[0] != 0 || savedPath[savedPathCount - 1] != savedGoalNode) {
        return false;
    }

    MazeHeading currentHeading = savedGoalHeading;
    for (int i = savedPathCount - 1; i > 0; --i) {
        int from = savedPath[i];
        int to = savedPath[i - 1];
        if (from < 0 || from >= savedNodeCount ||
            to < 0 || to >= savedNodeCount || from == to ||
            route.stepCount >= MAZE_MAX_RETURN_STEPS) {
            route = MazeReturnRoute{};
            return false;
        }

        int directionIndex = -1;
        for (int d = 0; d < 4; ++d) {
            if (savedNodes[from].neighbor[d] == to) {
                directionIndex = d;
                break;
            }
        }
        if (directionIndex < 0) {
            route = MazeReturnRoute{};
            return false;
        }

        const int opposite = (directionIndex + 2) % 4;
        const long distance = savedNodes[from].edgeDistance[directionIndex];
        if (savedNodes[to].neighbor[opposite] != from ||
            distance < 0 || savedNodes[to].edgeDistance[opposite] != distance) {
            route = MazeReturnRoute{};
            return false;
        }

        MazeHeading direction = static_cast<MazeHeading>(directionIndex);
        MazeReturnStep &step = route.steps[route.stepCount++];
        step.fromNode = from;
        step.toNode = to;
        step.direction = direction;
        step.turn = absoluteToRelative(currentHeading, direction);
        step.distanceTicks = distance;
        route.totalTicks += distance;
        currentHeading = direction;
    }
    return true;
}
