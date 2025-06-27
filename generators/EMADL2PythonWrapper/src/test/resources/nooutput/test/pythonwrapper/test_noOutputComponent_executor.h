/* (c) https://github.com/MontiCore/monticore */
#ifndef _TEST_NOOUTPUTCOMPONENT_EXECUTOR_H_
#define _TEST_NOOUTPUTCOMPONENT_EXECUTOR_H_
#include "armadillo"
#include "../test_noOutputComponent.h"

struct test_noOutputComponent_input {
    double inp1;
    arma::cube inp2;
};

class test_noOutputComponent_executor {
private:
    test_noOutputComponent instance;
public:
    void init();
    void execute(test_noOutputComponent_input input);
};
#endif
