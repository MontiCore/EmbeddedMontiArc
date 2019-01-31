#ifndef TESTA_CALCULATOR_ARGS2
#define TESTA_CALCULATOR_ARGS2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class TestA_calculator_args2{
public:
colvec input1;
colvec input2;
colvec input3;
int max;
void init()
{
input1=colvec(10);
input2=colvec(10);
input3=colvec(10);
}
void execute()
{
max = amax(input1-1)+10*amax(input2-1)+100*amax(input3-1);
}

};
#endif
