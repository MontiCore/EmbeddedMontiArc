/* (c) https://github.com/MontiCore/monticore */
#ifndef _TEST_NOINPUTCOMPONENT_EXECUTOR_H_
#define _TEST_NOINPUTCOMPONENT_EXECUTOR_H_
#include "armadillo"
#include "../test_noInputComponent.h"

struct test_noInputComponent_output {
    double out1;
    arma::cube out2;
};

class test_noInputComponent_executor {
private:
    test_noInputComponent instance;
public:
    void init();
    test_noInputComponent_output execute();
};
#endif
