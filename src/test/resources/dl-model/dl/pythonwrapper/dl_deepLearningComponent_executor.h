/* (c) https://github.com/MontiCore/monticore */
#ifndef _DL_DEEPLEARNINGCOMPONENT_EXECUTOR_H_
#define _DL_DEEPLEARNINGCOMPONENT_EXECUTOR_H_
#include "armadillo"
#include "../dl_deepLearningComponent.h"

struct dl_deepLearningComponent_input {
    arma::cube data;
};

struct dl_deepLearningComponent_output {
    arma::colvec predictions;
};

class dl_deepLearningComponent_executor {
private:
    dl_deepLearningComponent instance;
public:
    void init();
    dl_deepLearningComponent_output execute(dl_deepLearningComponent_input input);
};
#endif
