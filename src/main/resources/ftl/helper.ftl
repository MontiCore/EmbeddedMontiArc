<#macro matrix_to_array>
  val matrixToArray(Mat matrix) {
      val result = val::array();
      uword rows = matrix.nrows;
      uword cols = matrix.ncols;
      for (int i = 0; i < cols; i++) {
          val v = val::array();
          result.set(i, v);
          for (int j = 0; j < rows; j++) {
              v.set(j, matrix(i, j));
          }
      }
      return result;
  }
</#macro>

<#macro array_to_matrix>
  void copyArrayInMatrix(Mat matrix, val array2d) {
    for (int i = 0; i < sizeof(array2d); i++) {
      for (int j = 0; j < sizeof(array2d}.get(i)); j++) {
        matrix(i, j) = array2d.get(i).get(j);
      }
    }
  }
</#macro>
