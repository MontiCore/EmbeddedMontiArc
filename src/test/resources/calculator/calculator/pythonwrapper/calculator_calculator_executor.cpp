/* (c) https://github.com/MontiCore/monticore */
#include "calculator_calculator_executor.h"

void calculator_calculator_executor::init() {
    instance.init();
}

calculator_calculator_output calculator_calculator_executor::execute(calculator_calculator_input input) {
    calculator_calculator_output output;

    instance.a = input.a;
    instance.b = input.b;
    instance.c = input.c;

    instance.execute();

    output.result = instance.result;
    return output;
}
