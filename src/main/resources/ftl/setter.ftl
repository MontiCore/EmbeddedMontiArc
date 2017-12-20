<#macro simple methodName parameterType parameterName objectName variableName>
  void ${methodName} (${parameterType} ${parameterName}) {
  ${objectName}->${variableName} = ${parameterName};
  }
</#macro>

<#macro matrix methodName parameterType parameterName objectName variableName>
  void ${methodName} (${parameterType} ${parameterName}) {
    copyArrayInMatrix(${objectName}->${variableName}, ${parameterName});
  }
</#macro>

<#macro matrix_array methodName parameterType parameterName objectName variableName>
  void ${methodName} (${parameterType} ${parameterName}) {
    for (int i = 0; i < sizeof(${parameterName}); i++) {
      copyArrayInMatrix(${objectName}->${variableName}.get(i), ${parameterName}.get(i));
    }
  }
</#macro>