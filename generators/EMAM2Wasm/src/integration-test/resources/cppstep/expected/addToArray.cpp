/* (c) https://github.com/MontiCore/monticore */
#ifndef arraysize
#define arraysize(array) (sizeof(array)/sizeof(array[0]))
#endif
#include "models_addToArray.h"
#include <emscripten/bind.h>
#include <emscripten/val.h>

using namespace emscripten;

models_addToArray *model = new models_addToArray();

void init() {
  model->init();
}

void execute() {
  model->execute();
}

  val matrixToArray(mat matrix) {
    val result = val::array();
    uword rows = matrix.n_rows;
    uword cols = matrix.n_cols;
    for (int i = 0; i < rows; i++) {
      val v = val::array();
      result.set(i, v);
      for (int j = 0; j < cols; j++) {
        v.set(j, matrix(i, j));
      }
    }
    return result;
  }

  void copyArrayInMatrix(mat &matrix, val array2d) {
    for (int i = 0; i < array2d["length"].as<int>(); i++) {
      for (int j = 0; j < ((array2d[i])["length"]).as<int>(); j++) {
        matrix(i, j) = array2d[i][j].as<double>();
      }
    }
  }

//getters
  val getResult() {
    val result = val::array();
    for (int i = 0; i < arraysize(model->result); i++) {
      result.set(i, model->result[i]);
    }
    return result;
  }

//setters
  void setArray(val array) {
    for (int i = 0; i < array["length"].as<int>(); i++) {
  model->array[i] = array[i].as<double>();
    }
  }
  void setOffset(val offset) {
  model->offset = offset.as<double>();
  }

//emscripten bindings
EMSCRIPTEN_BINDINGS(my_module) {
  function("init", &init);
  function("execute", &execute);
    function("getResult", &getResult);
    function("setArray", &setArray);
    function("setOffset", &setOffset);
}
