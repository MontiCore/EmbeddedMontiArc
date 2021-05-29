/* (c) https://github.com/MontiCore/monticore */
#ifndef _TYPES_ALLTYPES_EXECUTOR_H_
#define _TYPES_ALLTYPES_EXECUTOR_H_
#include "armadillo"
#include "../types_allTypes.h"

struct types_allTypes_input {
    double q1;
    arma::colvec q2;
    arma::mat q3;
    arma::cube q4;
    int z1;
    arma::ivec z2;
    arma::imat z3;
    arma::icube z4;
    arma::ivec z5;
    bool b1;
};

struct types_allTypes_output {
    int result;
};

class types_allTypes_executor {
private:
    types_allTypes instance;
public:
    void init();
    types_allTypes_output execute(types_allTypes_input input);
};
#endif
