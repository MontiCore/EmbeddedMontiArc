/* (c) https://github.com/MontiCore/monticore */
#ifndef _ANY_COMPONENT_EXECUTOR_H_
#define _ANY_COMPONENT_EXECUTOR_H_
#include "armadillo"
#include "../any_component.h"

class any_component_executor {
private:
    any_component instance;
public:
    void init();
    void execute();
};
#endif
