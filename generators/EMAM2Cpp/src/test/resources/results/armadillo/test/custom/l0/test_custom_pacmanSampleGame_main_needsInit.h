/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_NEEDSINIT
#define TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_NEEDSINIT
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_custom_pacmanSampleGame_main_needsInit{
public:
bool out1;
double lastVal;
void init()
{
lastVal=true;
}
void execute()
{
out1 = lastVal;
lastVal = 0;
}

};
#endif
