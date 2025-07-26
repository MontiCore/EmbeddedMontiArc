<#-- (c) https://github.com/MontiCore/monticore -->
<#macro simple methodName objectName variableName>
  val ${methodName}() {
    return val(${objectName}->${variableName});
  }
</#macro>

<#macro array methodName objectName variableName>
  <#assign array = objectName+ "->" + variableName>
  val ${methodName}() {
    val result = val::array();
    for (int i = 0; i < arraysize(${array}); i++) {
      result.set(i, ${array}[i]);
    }
    return result;
  }
</#macro>

<#macro matrix methodName objectName variableName>
  val ${methodName}() {
    return matrixToArray(${objectName}->${variableName});
  }
</#macro>

<#macro matrix_array methodName objectName variableName>
  <#assign matrixarray = objectName+ "->" + variableName>
  val ${methodName}() {
    val result = val::array();
    for (int i = 0; i < arraysize(${matrixarray}); i++) {
      result.set(i, matrixToArray(${matrixarray}[i]));
    }
    return result;
  }
</#macro>
