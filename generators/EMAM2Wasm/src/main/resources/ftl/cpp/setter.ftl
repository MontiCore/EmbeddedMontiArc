<#-- (c) https://github.com/MontiCore/monticore -->
<#macro simple methodName parameterName objectName variableName variableType>
  void ${methodName}(val ${parameterName}) {
  ${objectName}->${variableName} = ${parameterName}.as<${variableType}>();
  }
</#macro>

<#macro array methodName parameterName objectName variableName variableType>
  void ${methodName}(val ${parameterName}) {
    for (int i = 0; i < ${parameterName}["length"].as<int>(); i++) {
  ${objectName}->${variableName}[i] = ${parameterName}[i].as<${variableType}>();
    }
  }
</#macro>

<#macro matrix methodName parameterName objectName variableName>
  void ${methodName}(val ${parameterName}) {
    copyArrayInMatrix(${objectName}->${variableName}, ${parameterName});
  }
</#macro>

<#macro matrix_array methodName parameterName objectName variableName>
  void ${methodName}(val ${parameterName}) {
    for (int i = 0; i < ${parameterName}["length"].as<int>(); i++) {
    copyArrayInMatrix(${objectName}->${variableName}[i], ${parameterName}[i]);
    }
  }
</#macro>
