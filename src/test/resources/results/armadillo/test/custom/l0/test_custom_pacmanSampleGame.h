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
main.execute();
}

};
#endif
