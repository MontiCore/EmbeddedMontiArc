/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_PACMANWORLDMANAGER_PACMANWORLDHANDLER
#define TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_PACMANWORLDMANAGER_PACMANWORLDHANDLER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler_movementHandler.h"
using namespace arma;
class test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler{
const int tilesMaxX = 16;
const int tilesMaxY = 16;
public:
bool moveUp;
bool moveDown;
bool moveLeft;
bool moveRight;
int pacmanPositionX;
int pacmanPositionY;
imat world;
int pacmanNextPositionX;
int pacmanNextPositionY;
test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler_movementHandler movementHandler;
void init()
{
world=imat(tilesMaxX,tilesMaxY);
movementHandler.init();
}
void execute()
{
}

};
#endif
