/* (c) https://github.com/MontiCore/monticore */
#ifndef _TEST_NOINPUTNOOUTPUTCOMPONENT_EXECUTOR_H_
#define _TEST_NOINPUTNOOUTPUTCOMPONENT_EXECUTOR_H_
#include "armadillo"
#include "../test_noInputNoOutputComponent.h"

class test_noInputNoOutputComponent_executor {
private:
    test_noInputNoOutputComponent instance;
public:
    void init();
    void execute();
};
#endif
