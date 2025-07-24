/* (c) https://github.com/MontiCore/monticore */
#include "models_model.h"
#include <emscripten/bind.h>
#include <emscripten/val.h>

using namespace emscripten;

models_model *model = new models_model();

void init() {
    model->init();
}

val getColor(int index) {
    val result = val::array();
    int i = 0;

    for (int x = 0; x < 640; x++) {
        for (int y = 0; y < 480; y++) {
            result.set(i++, val(model->rgba[index](x, y)));
        }
    }

    return result;
}

val matrixToArray(mat matrix) {
    val result = val::array();
    uword rows = matrix.n_rows;
    uword cols = matrix.n_cols;
    for (int i = 0; i < cols; i++) {
        val v = val::array();
        result.set(i, v);
        for (int j = 0; j < cols; j++) {
            v.set(j, model->rgba[1](i, j));
        }
    }
    return result;
}

EMSCRIPTEN_BINDINGS(my_module) {
        function("init", &init);
        function("getColor", &getColor);
}
