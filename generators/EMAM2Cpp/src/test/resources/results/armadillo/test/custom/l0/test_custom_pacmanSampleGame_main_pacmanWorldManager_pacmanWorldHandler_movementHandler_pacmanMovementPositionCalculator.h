/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_PACMANWORLDMANAGER_PACMANWORLDHANDLER_MOVEMENTHANDLER_PACMANMOVEMENTPOSITIONCALCULATOR
#define TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_PACMANWORLDMANAGER_PACMANWORLDHANDLER_MOVEMENTHANDLER_PACMANMOVEMENTPOSITIONCALCULATOR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler_movementHandler_pacmanMovementPositionCalculator{
const int tilesMaxX = 16;
const int tilesMaxY = 16;
public:
bool moveUp;
bool moveDown;
bool moveLeft;
bool moveRight;
int pacmanPositionX;
int pacmanPositionY;
int pacmanPositionMaxX;
int pacmanPositionMaxY;
int pacmanNextPositionX;
int pacmanNextPositionY;
void init()
{
}
void execute()
{
pacmanNextPositionX = pacmanPositionX;
pacmanNextPositionY = pacmanPositionY;
if((moveUp)){
if(((pacmanPositionY+1 < pacmanPositionMaxY))){
pacmanNextPositionY = pacmanNextPositionY+1;
}
}
else if((moveDown)){
if(((pacmanPositionY-1 > 0))){
pacmanNextPositionY = pacmanNextPositionY-1;
}
}
else if((moveLeft)){
if(((pacmanPositionX-1 > 0))){
pacmanNextPositionX = pacmanNextPositionX-1;
}
}
else if((moveRight)){
if(((pacmanPositionX+1 < pacmanPositionMaxX))){
pacmanNextPositionX = pacmanNextPositionX+1;
}
}
}

};
#endif
