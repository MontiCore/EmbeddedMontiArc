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

struct any_component_output {
    arma::imat anyName4;
    arma::vec anyName5;
};

class any_component_executor {
private:
    any_component instance;
public:
    void init();
    any_component_output execute(any_component_input input);
};
#endif
