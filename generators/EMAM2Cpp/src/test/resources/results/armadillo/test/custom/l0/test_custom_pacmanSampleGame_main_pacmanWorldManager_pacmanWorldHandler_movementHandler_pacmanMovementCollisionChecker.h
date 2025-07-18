/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_PACMANWORLDMANAGER_PACMANWORLDHANDLER_MOVEMENTHANDLER_PACMANMOVEMENTCOLLISIONCHECKER
#define TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_PACMANWORLDMANAGER_PACMANWORLDHANDLER_MOVEMENTHANDLER_PACMANMOVEMENTCOLLISIONCHECKER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler_movementHandler_pacmanMovementCollisionChecker{
const int tilesMaxX = 16;
const int tilesMaxY = 16;
public:
int pacmanNextPossibleTilePosition;
int pacmanOldTilePosition;
imat world;
int pacmanPositionX;
int pacmanPositionY;
int pacmanPossibleNextPositionX;
int pacmanPossibleNextPositionY;
bool canMove;
int pacmanChosenNextPositionX;
int pacmanChosenNextPositionY;
void init()
{
world=imat(tilesMaxX,tilesMaxY);
}
void execute()
{
int zvalue = 1;
zvalue = world(pacmanPossibleNextPositionX-1, pacmanPossibleNextPositionY-1);
if(((zvalue == 1))){
canMove = 0;
pacmanChosenNextPositionX = pacmanPositionX;
pacmanChosenNextPositionY = pacmanPositionY;
}
else {
canMove = 1;
pacmanChosenNextPositionX = pacmanPossibleNextPositionX;
pacmanChosenNextPositionY = pacmanPossibleNextPositionY;
}
}

};
#endif
