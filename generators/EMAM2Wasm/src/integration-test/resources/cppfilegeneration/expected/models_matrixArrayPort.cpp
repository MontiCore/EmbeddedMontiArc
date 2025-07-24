/* (c) https://github.com/MontiCore/monticore */
#ifndef arraysize
#define arraysize(array) (sizeof(array)/sizeof(array[0]))
#endif
#include "models_matrixArrayPort.h"
#include <emscripten/bind.h>
#include <emscripten/val.h>

using namespace emscripten;

models_matrixArrayPort *model = new models_matrixArrayPort();

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
val getOut1() {
	val result = val::array();
  for (int i = 0; i < arraysize(model->out1); i++) {
    result.set(i, matrixToArray(model->out1[i]));
  }
  return result;
}

//setters
void setIn1(val in1) {
  for (int i = 0; i < in1["length"].as<int>(); i++) {
		copyArrayInMatrix(model->in1[i], in1[i]);
	}
}

//emscripten bindings
EMSCRIPTEN_BINDINGS(my_module) {
        function("init", &init);
		function("execute", &execute);
        function("getOut1", &getOut1);
		function("setIn1", &setIn1);
}
