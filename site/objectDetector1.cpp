#ifndef arraysize
#define arraysize(array) (sizeof(array)/sizeof(array[0]))
#endif
#include "detection_objectDetector1.h"
#include <emscripten/bind.h>
#include <emscripten/val.h>

using namespace emscripten;

detection_objectDetector1 *model = new detection_objectDetector1();

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
  val getClusters() {
    return matrixToArray(model->clusters);
  }

//setters
  void setImgFront(val imgFront) {
    copyArrayInMatrix(model->imgFront, imgFront);
  }

//emscripten bindings
EMSCRIPTEN_BINDINGS(my_module) {
  function("init", &init);
  function("execute", &execute);
    function("getClusters", &getClusters);
    function("setImgFront", &setImgFront);
}