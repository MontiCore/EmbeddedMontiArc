/* (c) https://github.com/MontiCore/monticore */
#ifndef _ANY_COMPONENT_EXECUTOR_H_
#define _ANY_COMPONENT_EXECUTOR_H_
#include "armadillo"
#include "../any_component.h"

struct any_component_input {
    int anyName1;
    arma::colvec anyName2;
    arma::cube anyName3;
};

class any_component_executor {
private:
    any_component instance;
public:
    void init();
    void execute(any_component_input input);
};
#endif
