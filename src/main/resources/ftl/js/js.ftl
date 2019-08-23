<#-- (c) https://github.com/MontiCore/monticore -->
<#ftl strip_whitespace=true>
var Module = {
  'print': function (text) { console.log('stdout: ' + text) },
  'printErr': function (text) { console.log('stderr: ' + text) },
  onRuntimeInitialized: function () { Module.init(); }
};

function init() {
  Module.init();
}

function execute() {
  Module.execute();
}

<#list getters as getter>
  function ${getter.methodName}() {
    return math.format(Module.${getter.delegateMethodName}(), {notation: 'fixed'})
  <#if getter.unit?has_content>.concat(" ${getter.unit}")</#if>;
  }
</#list>

<#list setters as setter>
  <#assign varName = "value">
  <#assign varNumber = varName + "_num">
  <#assign elementName = "e">
  <#assign elementNumber = elementName + "_num">
  <#assign lowerBoundVar = "lower">
  <#assign upperBoundVar = "upper">
  <#assign portName = setter.parameterName>
  <#assign typeUnit = "Unit">
  <#assign typeNumber = "number">

<#--prefix parameter with "_" so ports can be named Javascript keywords (e.g. "undefined", "var")-->
  function ${setter.methodName}(_${setter.parameterName}) {
  var ${varName} = math.eval(_${setter.parameterName});
  <#if setter.lowerBoundValue?has_content>
    <#if setter.lowerBoundUnit?has_content>
      var ${lowerBoundVar} = math.eval("${setter.lowerBoundValue} ${setter.lowerBoundUnit}").toSI().toNumber();
    <#else>
      var ${lowerBoundVar} = ${setter.lowerBoundValue};
    </#if>
  </#if>
  <#if setter.upperBoundValue?has_content>
    <#if setter.upperBoundUnit?has_content>
      var ${upperBoundVar} = math.eval("${setter.upperBoundValue} ${setter.upperBoundUnit}").toSI().toNumber();
    <#else>
      var ${upperBoundVar} = ${setter.upperBoundValue};
    </#if>
  </#if>

  if (${varName} === undefined) {
    throw "${portName}: Could not evaluate input";
  }

  <#if setter.dimension?has_content>
    //check dimension
    var dim = math.matrix([${setter.dimension?join(", ")}]);
    if (!math.deepEqual(math.size(${varName}), dim)) {
      throw "${portName}: Input has dimension " + math.size(${varName}) + " but expected " + dim;
    }

    var array = [];
    <#assign dimensions = setter.dimension?size>
    <@forloop index = 0 dim = setter.dimension times = dimensions - 1>
      var ${elementName} = ${varName}.get([<#list 0..<dimensions as i>i${i}<#sep>,</#list>]);

      <@checkUnit identifier=portName unit=setter.lowerBoundUnit!"" var=elementName/>
      <#if setter.lowerBoundUnit?has_content>
        var ${elementNumber} = ${elementName}.toSI().toNumber();
      <#else>
        var ${elementNumber} = ${elementName};
      </#if>
      <@checkRange identifier=portName var=elementNumber lowerBound=setter.lowerBoundValue!"" upperBound=setter.upperBoundValue!""/>
      array<@arrayindex n=dimensions/> = ${elementNumber};
    </@forloop>
    Module.${setter.delegateMethodName}(array);
  <#else>
    <@checkType identifier=portName var=varName type=setter.lowerBoundUnit?has_content?then(typeUnit, typeNumber)/>
    <@checkUnit identifier=portName unit=setter.lowerBoundUnit!"" var=varName/>
    <#if setter.lowerBoundUnit?has_content>
      var ${varNumber} = ${varName}.toSI().toNumber();
    <#else>
      var ${varNumber} = ${varName};
    </#if>
    <@checkRange identifier=portName var=varNumber lowerBound=setter.lowerBoundValue!"" upperBound=setter.upperBoundValue!""/>
    Module.${setter.delegateMethodName}(${varNumber});
  </#if>
}
</#list>

<#macro forloop index dim times>
  for (var i${index} = 0; i${index} < ${dim[index]}; i${index}++) {
  <#if times gt 0>
    array<@arrayindex n=index+1/> = [];
    <@forloop index=index+1 dim=dim times=times-1><#nested></@forloop>
  <#else>
    <#nested>
  </#if>
  }
</#macro>

<#macro arrayindex n>
  <#list 0..<n as i>[i${i}]</#list>
</#macro>

<#macro checkUnit identifier unit var>
  <#if unit?has_content>
    //check unit
    var expectedUnit = math.eval("${unit}");
    if (math.typeof(expectedUnit) !== math.typeof(${var}) || !expectedUnit.equalBase(${var})) {
      throw "${identifier}: Expected unit ${unit}";
    }
  </#if>
</#macro>

<#macro checkRange identifier var lowerBound upperBound>
  <#if lowerBound?has_content || upperBound?has_content>
    //check range
    <#if lowerBound?has_content>
  if (math.smaller(${var}, ${lowerBoundVar})) {
        throw "${identifier}: Value " + ${var} + " out of range";
      }
    </#if>
    <#if upperBound?has_content>
  if (math.larger(${var}, ${upperBoundVar})) {
        throw "${identifier}: Value " + ${var} + " out of range";
      }
    </#if>
  </#if>
</#macro>

<#macro checkType identifier var type>
  //check type
  if (math.typeof(${var}) !== "${type}") {
    throw "${identifier}: Expected type ${type}";
  }
</#macro>
