#ifndef TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_DELAYEDPOSX
#define TEST_CUSTOM_PACMANSAMPLEGAME_MAIN_DELAYEDPOSX
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_custom_pacmanSampleGame_main_delayedPosX{
public:
double dvalue;
double input;
double output;
double delayed;
void init(double dvalue)
{
this->dvalue=dvalue;
delayed=0;
}
void execute()
{
output = 0;
dvalue = input;
}
void output()
{
Q delayed_TEMP_ = copy(delayed)
output = 0;
dvalue = input;
delayed=delayed_TEMP_;;
}
void update()
{
output = 0;
dvalue = input;
}

};
#endif
