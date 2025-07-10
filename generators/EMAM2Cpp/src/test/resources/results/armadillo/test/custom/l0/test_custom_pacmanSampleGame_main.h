/* (c) https://github.com/MontiCore/monticore */
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
#include "test_custom_pacmanSampleGame_main_delayedPosX.h"
#include "test_custom_pacmanSampleGame_main_delayedPosY.h"
using namespace arma;
class test_custom_pacmanSampleGame_main{
public:
test_custom_pacmanSampleGame_main_needsInit needsInit;
test_custom_pacmanSampleGame_main_pacmanInit pacmanInit;
test_custom_pacmanSampleGame_main_stateSaver stateSaver;
test_custom_pacmanSampleGame_main_pacmanInputControl pacmanInputControl;
test_custom_pacmanSampleGame_main_stubby stubby;
test_custom_pacmanSampleGame_main_pacmanWorldManager pacmanWorldManager;
test_custom_pacmanSampleGame_main_delayedPosX delayedPosX;
test_custom_pacmanSampleGame_main_delayedPosY delayedPosY;
void init()
{
needsInit.init();
pacmanInit.init();
stateSaver.init();
pacmanInputControl.init();
stubby.init();
pacmanWorldManager.init();
delayedPosX.init(0);
delayedPosY.init(0);
}
void execute()
{
}
};
#endif
