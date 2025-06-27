/* (c) https://github.com/MontiCore/monticore */
#include "dl_deepLearningComponent_executor.h"

void dl_deepLearningComponent_executor::init() {
    instance.init();
}

dl_deepLearningComponent_output dl_deepLearningComponent_executor::execute(dl_deepLearningComponent_input input) {
    dl_deepLearningComponent_output output;

    instance.data = input.data;

    instance.execute();

    output.predictions = instance.predictions;
    return output;
}
