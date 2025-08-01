<#-- (c) https://github.com/MontiCore/monticore -->
<#macro matrix_to_array>
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
</#macro>

<#macro array_to_matrix>
  void copyArrayInMatrix(mat &matrix, val array2d) {
    for (int i = 0; i < array2d["length"].as<int>(); i++) {
      for (int j = 0; j < ((array2d[i])["length"]).as<int>(); j++) {
        matrix(i, j) = array2d[i][j].as<double>();
      }
    }
  }
</#macro>
