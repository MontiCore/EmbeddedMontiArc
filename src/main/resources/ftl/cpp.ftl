<#ftl strip_whitespace=true>
<#import "getter.ftl" as g>
<#import "setter.ftl" as s>
<#import "helper.ftl" as h>
#include "${main_class}.h"
#include
<emscripten/bind.h>
#include
<emscripten/val.h>

using namespace emscripten;

<#assign object_name = "*_model">
${main_class} ${object_name} = new ${main_class}();

void init() {
  model->init();
}

//getters
<#list getters as getter>
  <#if getter.datatype == "matrix">
    <#if getter.type == "scalar">
      <@g.matrix methodName=getter.methodName objectName=object_name
      variableName=getter.variableName>
      </@g.matrix>
    <#elseif getter.type == "array">
      <@g.matrix_array methodName=getter.methodName objectName=object_name
      variableName=getter.variableName>
      </@g.matrix_array>
    </#if>
  <#elseif getter.datatype == "primitive">
    <@g.simple methodName=getter.methodName objectName=object_name
    variableName=getter.variableName>
    </@g.simple>
  </#if>
</#list>

//setters
<#list setters as setter>
  <#if setter.datatype == "matrix">
    <#if setter.type == "scalar">
      <@s.matrix methodName=setter.methodName parameterType=setter.parameterType
      parameterName=setter.parameterName objectName=object_name variableName=setter.variableName>
      </@s.matrix>
    <#elseif setter.type == "array">
      <@s.matrix_array methodName=setter.methodName parameterType=setter.parameterType
      parameterName=setter.parameterName objectName=object_name variableName=setter.variableName>
      </@s.matrix_array>
    </#if>
  <#elseif setter.datatype == "primitive">
    <@s.simple methodName=setter.methodName parameterType=setter.parameterType
    parameterName=setter.parameterName objectName=object_name variableName=setter.variableName>
    </@s.simple>
  </#if>

  void ${setter.getMethodName()} (${setter.getVariableName()}) {
  ${object_name}->${setter.getVariableName()} = ${setter.getVariableName()};
  }
</#list>

<@h.matrix_to_array>
</@h.matrix_to_array>

<@h.array_to_matrix>
</@h.array_to_matrix>

//emscripten bindings
EMSCRIPTEN_BINDINGS(my_module) {
  function("init", &init);
  function("execute", &execute);
<#list getters+setters as function>
    function("${function.methodName}", &${function.methodName});
</#list>
}