#ifndef CNNCALCULATOR_CONNECTOR_CAL
#define CNNCALCULATOR_CONNECTOR_CAL
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "cNNCalculator_connector_number1_ones.h"
#include "cNNCalculator_connector_number1_tens.h"
#include "cNNCalculator_connector_number1_hundreds.h"
#include "cNNCalculator_connector_number2_ones.h"
#include "cNNCalculator_connector_number2_tens.h"
#include "cNNCalculator_connector_number2_hundreds.h"
#include "cNNCalculator_connector_number1.h"
#include "cNNCalculator_connector_number2.h"
#include "cNNCalculator_connector_add.h"
using namespace arma;
class cNNCalculator_connector_cal{
public:
colvec in1_1;
colvec in1_2;
colvec in1_3;
colvec in2_1;
colvec in2_2;
colvec in2_3;
int out1;
cNNCalculator_connector_number1_ones number1_ones;
cNNCalculator_connector_number1_tens number1_tens;
cNNCalculator_connector_number1_hundreds number1_hundreds;
cNNCalculator_connector_number2_ones number2_ones;
cNNCalculator_connector_number2_tens number2_tens;
cNNCalculator_connector_number2_hundreds number2_hundreds;
cNNCalculator_connector_number1 number1;
cNNCalculator_connector_number2 number2;
cNNCalculator_connector_add add;
void init()
{
in1_1=colvec(10);
in1_2=colvec(10);
in1_3=colvec(10);
in2_1=colvec(10);
in2_2=colvec(10);
in2_3=colvec(10);
number1_ones.init();
number1_tens.init();
number1_hundreds.init();
number2_ones.init();
number2_tens.init();
number2_hundreds.init();
number1.init();
number2.init();
add.init();
}
void execute()
{
number1_hundreds.inputVector = in1_1;
number1_hundreds.execute();
number1_tens.inputVector = in1_2;
number1_tens.execute();
number1_ones.inputVector = in1_3;
number1_ones.execute();
number1.ones = number1_ones.maxIndex;
number1.tens = number1_tens.maxIndex;
number1.hundreds = number1_hundreds.maxIndex;
number1.execute();
number2_hundreds.inputVector = in2_1;
number2_hundreds.execute();
number2_tens.inputVector = in2_2;
number2_tens.execute();
number2_ones.inputVector = in2_3;
number2_ones.execute();
number2.ones = number2_ones.maxIndex;
number2.tens = number2_tens.maxIndex;
number2.hundreds = number2_hundreds.maxIndex;
number2.execute();
add.num1 = number1.number;
add.num2 = number2.number;
add.execute();
out1 = add.sum;
}

};
#endif
