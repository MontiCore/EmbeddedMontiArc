#ifndef TEST_CUSTOM_PACMANSAMPLEGAME_MAIN
#define TEST_CUSTOM_PACMANSAMPLEGAME_MAIN
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "test_custom_pacmanSampleGame_main_needsInit.h"
#include "test_custom_pacmanSampleGame_main_pacmanInit.h"
#include "test_custom_pacmanSampleGame_main_stateSaver.h"
#include "test_custom_pacmanSampleGame_main_pacmanInputControl.h"
#include "test_custom_pacmanSampleGame_main_stubby.h"
#include "test_custom_pacmanSampleGame_main_pacmanWorldManager.h"
using namespace arma;
class test_custom_pacmanSampleGame_main{
public:
test_custom_pacmanSampleGame_main_needsInit needsInit;
test_custom_pacmanSampleGame_main_pacmanInit pacmanInit;
test_custom_pacmanSampleGame_main_stateSaver stateSaver;
test_custom_pacmanSampleGame_main_pacmanInputControl pacmanInputControl;
test_custom_pacmanSampleGame_main_stubby stubby;
test_custom_pacmanSampleGame_main_pacmanWorldManager pacmanWorldManager;
void init()
{
needsInit.init();
pacmanInit.init();
stateSaver.init();
pacmanInputControl.init();
stubby.init();
pacmanWorldManager.init();
}
void execute()
{
needsInit.execute();
pacmanInit.exec = needsInit.out1;
pacmanInit.execute();
pacmanInputControl.execute();
stubby.moveDown = pacmanInputControl.moveDown;
stubby.moveUp = pacmanInputControl.moveUp;
stubby.moveLeft = pacmanInputControl.moveLeft;
stubby.moveRight = pacmanInputControl.moveRight;
stubby.execute();
stateSaver.needsInit = needsInit.out1;
stateSaver.positionX = pacmanWorldManager.pacmanNextPositionX;
stateSaver.positionY = pacmanWorldManager.pacmanNextPositionY;
stateSaver.pacmanPositionX = pacmanWorldManager.pacmanNextPositionX;
stateSaver.pacmanPositionY = pacmanWorldManager.pacmanNextPositionY;
stateSaver.execute();
pacmanWorldManager.needsInit = needsInit.out1;
pacmanWorldManager.moveDown = pacmanInputControl.moveDown;
pacmanWorldManager.moveUp = pacmanInputControl.moveUp;
pacmanWorldManager.moveLeft = pacmanInputControl.moveLeft;
pacmanWorldManager.moveRight = pacmanInputControl.moveRight;
pacmanWorldManager.pacmanPositionX = stateSaver.pacmanPositionX;
pacmanWorldManager.pacmanPositionY = stateSaver.pacmanPositionY;
pacmanWorldManager.execute();
}

};
#endif
