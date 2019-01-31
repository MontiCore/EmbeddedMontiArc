#ifndef TESTA_CALCULATOR
#define TESTA_CALCULATOR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "TestA_calculator_args1.h"
#include "TestA_calculator_args2.h"
#include "TestA_calculator_add.h"
using namespace arma;
class TestA_calculator{
public:
colvec in1_1;
colvec in1_2;
colvec in1_3;
colvec in2_1;
colvec in2_2;
colvec in2_3;
int out1;
TestA_calculator_args1 args1;
TestA_calculator_args2 args2;
TestA_calculator_add add;
void init()
{
in1_1=colvec(10);
in1_2=colvec(10);
in1_3=colvec(10);
in2_1=colvec(10);
in2_2=colvec(10);
in2_3=colvec(10);
args1.init();
args2.init();
add.init();
}
void execute()
{
args1.input1 = in1_1;
args1.input2 = in1_2;
args1.input3 = in1_3;
args1.execute();
args2.input1 = in2_1;
args2.input2 = in2_2;
args2.input3 = in2_3;
args2.execute();
add.num1 = args1.max;
add.num2 = args2.max;
add.execute();
out1 = add.sum;
}

};
#endif
