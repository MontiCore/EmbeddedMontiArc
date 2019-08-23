/* (c) https://github.com/MontiCore/monticore */
#include "any_component_executor.h"

void any_component_executor::init() {
    instance.init();
}

any_component_output any_component_executor::execute() {
    any_component_output output;


    instance.execute();

    output.anyName4 = instance.anyName4;
    output.anyName5 = instance.anyName5;
    return output;
}
