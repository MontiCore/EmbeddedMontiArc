#ifndef TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_DELAYEDPOSY
#define TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_DELAYEDPOSY
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_custom_pacmanSampleGame_main_delayedPosY{
public:
double dvalue;
double input;
double output;
void init(double dvalue)
{
this->dvalue=dvalue;
}
void execute()
{
double delayed = 0;
output = 0;
dvalue = input;
}
void output()
{
double delayed = 0;
output = 0;
dvalue = input;
}
void update()
{
double delayed = 0;
output = 0;
dvalue = input;
}

};
#endif
