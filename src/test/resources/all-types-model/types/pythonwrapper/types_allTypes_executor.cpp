/* (c) https://github.com/MontiCore/monticore */
#include "types_allTypes_executor.h"

void types_allTypes_executor::init() {
    instance.init();
}

types_allTypes_output types_allTypes_executor::execute(types_allTypes_input input) {
    types_allTypes_output output;

    instance.q1 = input.q1;
    instance.q2 = input.q2;
    instance.q3 = input.q3;
    instance.q4 = input.q4;
    instance.z1 = input.z1;
    instance.z2 = input.z2;
    instance.z3 = input.z3;
    instance.z4 = input.z4;
    instance.z5 = input.z5;
    instance.b1 = input.b1;

    instance.execute();

    output.result = instance.result;
    return output;
}
