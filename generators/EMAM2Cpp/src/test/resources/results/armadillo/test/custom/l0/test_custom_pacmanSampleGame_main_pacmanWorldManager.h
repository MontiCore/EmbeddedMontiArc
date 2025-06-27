/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_PACMANWORLDMANAGER
#define TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_PACMANWORLDMANAGER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler.h"
using namespace arma;
class test_custom_pacmanSampleGame_main_pacmanWorldManager{
const int tilesMaxX = 16;
const int tilesMaxY = 16;
public:
bool needsInit;
bool moveUp;
bool moveDown;
bool moveLeft;
bool moveRight;
int pacmanPositionX;
int pacmanPositionY;
imat world;
int pacmanNextPositionX;
int pacmanNextPositionY;
test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler pacmanWorldHandler;
void init()
{
world=imat(tilesMaxX,tilesMaxY);
pacmanWorldHandler.init();
}
void execute()
{
}

};
#endif
