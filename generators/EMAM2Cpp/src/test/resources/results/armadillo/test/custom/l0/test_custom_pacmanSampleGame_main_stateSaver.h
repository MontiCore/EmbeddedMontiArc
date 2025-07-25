/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_STATESAVER
#define TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_STATESAVER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_custom_pacmanSampleGame_main_stateSaver{
public:
bool needsInit;
int positionX;
int positionY;
int pacmanPositionX;
int pacmanPositionY;
void init()
{
}
void execute()
{
if((needsInit)){
pacmanPositionX = 0;
pacmanPositionY = 0;
}
else {
pacmanPositionX = positionX;
pacmanPositionY = positionY;
}
}

};
#endif
