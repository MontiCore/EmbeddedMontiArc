/* (c) https://github.com/MontiCore/monticore */
#ifndef _CALCULATOR_CALCULATOR_EXECUTOR_H_
#define _CALCULATOR_CALCULATOR_EXECUTOR_H_
#include "armadillo"
#include "../calculator_calculator.h"

struct calculator_calculator_input {
    arma::colvec a;
    arma::colvec b;
    int c;
};

struct calculator_calculator_output {
    double result;
};

class calculator_calculator_executor {
private:
    calculator_calculator instance;
public:
    void init();
    calculator_calculator_output execute(calculator_calculator_input input);
};
#endif
