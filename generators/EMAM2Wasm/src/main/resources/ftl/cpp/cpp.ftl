<#-- (c) https://github.com/MontiCore/monticore -->
<#ftl strip_whitespace=true>
<#import "getter.ftl" as g>
<#import "setter.ftl" as s>
<#import "helper.ftl" as h>
#ifndef arraysize
#define arraysize(array) (sizeof(array)/sizeof(array[0]))
#endif
#include "${main_class}.h"
#include <emscripten/bind.h>
#include <emscripten/val.h>

using namespace emscripten;

<#assign object_name = "model">
${main_class} *${object_name} = new ${main_class}();

void init() {
  model->init();
}

void execute() {
  model->execute();
}

<@h.matrix_to_array>
</@h.matrix_to_array>

<@h.array_to_matrix>
</@h.array_to_matrix>

//getters
<#list getters as getter>
  <#if getter.datatype == "MATRIX">
    <#if getter.type == "SCALAR">
      <@g.matrix methodName=getter.methodName objectName=object_name
      variableName=getter.variableName>
      </@g.matrix>
    <#elseif getter.type == "ARRAY">
      <@g.matrix_array methodName=getter.methodName objectName=object_name
      variableName=getter.variableName>
      </@g.matrix_array>
    </#if>
  <#elseif getter.datatype == "PRIMITIVE">
    <#if getter.type == "SCALAR">
      <@g.simple methodName=getter.methodName objectName=object_name
      variableName=getter.variableName>
      </@g.simple>
    <#elseif getter.type == "ARRAY">
      <@g.array methodName=getter.methodName objectName=object_name
      variableName=getter.variableName>
      </@g.array>
    </#if>
  </#if>
</#list>

//setters
<#list setters as setter>
  <#if setter.datatype == "MATRIX">
    <#if setter.type == "SCALAR">
      <@s.matrix methodName=setter.methodName parameterName=setter.parameterName
      objectName=object_name variableName=setter.variableName>
      </@s.matrix>
    <#elseif setter.type == "ARRAY">
      <@s.matrix_array methodName=setter.methodName parameterName=setter.parameterName
      objectName=object_name variableName=setter.variableName>
      </@s.matrix_array>
    </#if>
  <#elseif setter.datatype == "PRIMITIVE">
    <#if setter.type == "SCALAR">
      <@s.simple methodName=setter.methodName parameterName=setter.parameterName
      objectName=object_name variableName=setter.variableName variableType=setter.variableType>
      </@s.simple>
    <#elseif setter.type == "ARRAY">
      <@s.array methodName=setter.methodName parameterName=setter.parameterName
      objectName=object_name variableName=setter.variableName variableType=setter.variableType>
      </@s.array>
    </#if>
  </#if>
</#list>

//emscripten bindings
EMSCRIPTEN_BINDINGS(my_module) {
  function("init", &init);
  function("execute", &execute);
<#list getters+setters as function>
    function("${function.methodName}", &${function.methodName});
</#list>
}
