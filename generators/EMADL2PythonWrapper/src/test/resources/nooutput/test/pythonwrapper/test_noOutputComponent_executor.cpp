/* (c) https://github.com/MontiCore/monticore */
#include "test_noOutputComponent_executor.h"

void test_noOutputComponent_executor::init() {
    instance.init();
}

void test_noOutputComponent_executor::execute(test_noOutputComponent_input input) {
    instance.inp1 = input.inp1;
    instance.inp2 = input.inp2;

    instance.execute();
}
