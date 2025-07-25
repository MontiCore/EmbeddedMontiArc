/* (c) https://github.com/MontiCore/monticore */
#include "any_component_executor.h"

void any_component_executor::init() {
    instance.init();
}

void any_component_executor::execute(any_component_input input) {
    instance.anyName1 = input.anyName1;
    instance.anyName2 = input.anyName2;
    instance.anyName3 = input.anyName3;

    instance.execute();

}
