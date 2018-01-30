#ifndef TESTS_A_COMPA
#define TESTS_A_COMPA
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class tests_a_compA{
public:
double rosIn;
double noRosIn;
double rosOut;
double noRosOut;
void init()
{
}
void execute()
{
rosOut = rosIn;
}

};
#endif
