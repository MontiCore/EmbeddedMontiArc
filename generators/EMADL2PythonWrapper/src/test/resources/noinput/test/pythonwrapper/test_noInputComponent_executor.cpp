/* (c) https://github.com/MontiCore/monticore */
#include "test_noInputComponent_executor.h"

void test_noInputComponent_executor::init() {
    instance.init();
}

test_noInputComponent_output test_noInputComponent_executor::execute() {
    test_noInputComponent_output output;
    instance.execute();

    output.out1 = instance.out1;
    output.out2 = instance.out2;
    return output;
}
