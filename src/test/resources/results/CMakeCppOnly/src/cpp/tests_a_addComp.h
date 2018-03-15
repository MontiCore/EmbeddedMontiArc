#ifndef TESTS_A_ADDCOMP
#define TESTS_A_ADDCOMP
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class tests_a_addComp{
public:
double in1;
double in2;
double out1;
void init()
{
}
void execute()
{
out1 = in1+in2;
}

};
#endif
