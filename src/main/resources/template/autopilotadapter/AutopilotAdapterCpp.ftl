<#include "/Common.ftl">
#include "AutopilotAdapter.h"

#include "./${viewModel.mainModelName}.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "armadillo.h"


void copy_double_array_to_mat(double *data, int size, mat &dest){
    for (int i=0; i<size; ++i) {
        dest(0,i) = data[i];
    }
}

${viewModel.mainModelName} AUTOPILOT_INSTANCE;

int input_count = ${viewModel.inputCount};
int output_count = ${viewModel.outputCount};

const char *input_names[] = { ${viewModel.inputNames}
};
const char *output_names[] = { ${viewModel.outputNames}
};
const char *input_types[] = { ${viewModel.inputTypes}
};
const char *output_types[] = { ${viewModel.outputTypes}
};

extern "C" {

EXPORT int get_input_count(){
    return input_count;
}
EXPORT const char *get_input_name(int id){
    return input_names[id];
}
EXPORT const char *get_input_type(int id){
    return input_types[id];
}

EXPORT int get_output_count(){
    return output_count;
}
EXPORT const char *get_output_name(int id){
    return output_names[id];
}
EXPORT const char *get_output_type(int id){
    return output_types[id];
}

EXPORT void init(){
    srand(time(NULL));
    AUTOPILOT_INSTANCE.init();
}
EXPORT void execute(){
    AUTOPILOT_INSTANCE.execute();
}

${viewModel.functionImplementations}


}