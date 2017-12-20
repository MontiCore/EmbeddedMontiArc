<#macro simple methodName objectName variableName>
  val ${methodName} () {
    return ${objectName}->${variableName};
  }
</#macro>

<#macro matrix methodName objectName variableName>
  val ${methodName} () {
    return matrixToArray(${objectName}->${variableName});
  }
</#macro>

<#macro matrix_array methodName objectName variableName>
  <#assign matrixarray = objectName+ "->" + variableName>
  val ${methodName} () {
    val result = val::array();
    for (int i = 0; i < sizeof(${matrixarray}); i++) {
      result.set(i, matrixToArray(${matrixarray});
    }
    return result;
  }
</#macro>