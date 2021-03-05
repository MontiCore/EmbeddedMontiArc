/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_CUSTOM_PACMANSAMPLEGAME
#define TEST_CUSTOM_PACMANSAMPLEGAME
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "test_custom_pacmanSampleGame_main.h"
using namespace arma;
class test_custom_pacmanSampleGame{
public:
test_custom_pacmanSampleGame_main main;
void init()
{
main.init();
}
void execute()
{
main.needsInit.execute();
main.pacmanInit.exec = main.needsInit.out1;
main.stateSaver.needsInit = main.needsInit.out1;
main.pacmanInputControl.execute();
main.stubby.moveDown = main.pacmanInputControl.moveDown;
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementPositionCalculator.moveDown = main.pacmanInputControl.moveDown;
main.stubby.moveUp = main.pacmanInputControl.moveUp;
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementPositionCalculator.moveUp = main.pacmanInputControl.moveUp;
main.stubby.moveLeft = main.pacmanInputControl.moveLeft;
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementPositionCalculator.moveLeft = main.pacmanInputControl.moveLeft;
main.stubby.moveRight = main.pacmanInputControl.moveRight;
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementPositionCalculator.moveRight = main.pacmanInputControl.moveRight;
main.delayedPosX.output();
main.stateSaver.positionX = main.delayedPosX.output;
main.delayedPosY.output();
main.stateSaver.positionY = main.delayedPosY.output;
main.pacmanInit.execute();
main.stateSaver.execute();
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementPositionCalculator.pacmanPositionX = main.stateSaver.pacmanPositionX;
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementCollisionChecker.pacmanPositionX = main.stateSaver.pacmanPositionX;
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementPositionCalculator.pacmanPositionY = main.stateSaver.pacmanPositionY;
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementCollisionChecker.pacmanPositionY = main.stateSaver.pacmanPositionY;
main.stubby.execute();
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementPositionCalculator.execute();
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementCollisionChecker.pacmanPossibleNextPositionX = main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementPositionCalculator.pacmanNextPositionX;
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementCollisionChecker.pacmanPossibleNextPositionY = main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementPositionCalculator.pacmanNextPositionY;
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementCollisionChecker.execute();
main.delayedPosX.input = main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementCollisionChecker.pacmanChosenNextPositionX;
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.controller.pacmanPositionX = main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementCollisionChecker.pacmanChosenNextPositionX;
main.delayedPosY.input = main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementCollisionChecker.pacmanChosenNextPositionY;
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.controller.pacmanPositionY = main.pacmanWorldManager.pacmanWorldHandler.movementHandler.pacmanMovementCollisionChecker.pacmanChosenNextPositionY;
main.pacmanWorldManager.pacmanWorldHandler.movementHandler.controller.execute();
main.delayedPosX.update();
main.delayedPosY.update();
}

};
#endif
