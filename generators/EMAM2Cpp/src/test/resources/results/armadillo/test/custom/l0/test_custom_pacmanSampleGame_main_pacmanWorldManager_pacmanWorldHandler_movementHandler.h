/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_PACMANWORLDMANAGER_PACMANWORLDHANDLER_MOVEMENTHANDLER
#define TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_PACMANWORLDMANAGER_PACMANWORLDHANDLER_MOVEMENTHANDLER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler_movementHandler_pacmanMovementPositionCalculator.h"
#include "test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler_movementHandler_pacmanMovementCollisionChecker.h"
#include "test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler_movementHandler_controller.h"
using namespace arma;
class test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler_movementHandler{
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
imat world;
int pacmanNextPositionX;
int pacmanNextPositionY;
test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler_movementHandler_pacmanMovementPositionCalculator pacmanMovementPositionCalculator;
test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler_movementHandler_pacmanMovementCollisionChecker pacmanMovementCollisionChecker;
test_custom_pacmanSampleGame_main_pacmanWorldManager_pacmanWorldHandler_movementHandler_controller controller;
void init()
{
world=imat(tilesMaxX,tilesMaxY);
pacmanMovementPositionCalculator.init();
pacmanMovementCollisionChecker.init();
controller.init();
}
void execute()
{
pacmanMovementPositionCalculator.moveUp = moveUp;
pacmanMovementPositionCalculator.moveDown = moveDown;
pacmanMovementPositionCalculator.moveLeft = moveLeft;
pacmanMovementPositionCalculator.moveRight = moveRight;
pacmanMovementPositionCalculator.pacmanPositionX = pacmanPositionX;
pacmanMovementPositionCalculator.pacmanPositionY = pacmanPositionY;
pacmanMovementPositionCalculator.execute();
pacmanMovementCollisionChecker.world = world;
pacmanMovementCollisionChecker.pacmanPositionX = pacmanPositionX;
pacmanMovementCollisionChecker.pacmanPositionY = pacmanPositionY;
pacmanMovementCollisionChecker.pacmanPossibleNextPositionX = pacmanMovementPositionCalculator.pacmanNextPositionX;
pacmanMovementCollisionChecker.pacmanPossibleNextPositionY = pacmanMovementPositionCalculator.pacmanNextPositionY;
pacmanMovementCollisionChecker.execute();
controller.pacmanPositionX = pacmanMovementCollisionChecker.pacmanChosenNextPositionX;
controller.pacmanPositionY = pacmanMovementCollisionChecker.pacmanChosenNextPositionY;
controller.execute();
pacmanNextPositionX = pacmanMovementCollisionChecker.pacmanChosenNextPositionX;
pacmanNextPositionY = pacmanMovementCollisionChecker.pacmanChosenNextPositionY;
}

};
#endif