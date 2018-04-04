<#ftl strip_whitespace=true>
var Module = {
  'print': function (text) { console.log('stdout: ' + text) },
  'printErr': function (text) { console.log('stderr: ' + text) },
  onRuntimeInitialized: function () { Module.init(); }
};

function execute() {
  Module.execute();
}

<#list getters as getter>
function ${getter.methodName}() {
  return math.format(Module.${getter.delegateMethodName}(), {notation: 'fixed'})
  <#if getter.unit??>.concat(" ${getter.unit}")</#if>;
}
</#list>

<#list setters as setter>
  <#assign varName = "value">
  <#assign varNumber = varName + "_num">
  <#assign elementName = "e">
  <#assign elementNumber = elementName + "_num">
  <#assign lowerBound = setter.lowerBound!"">
  <#assign upperBound = setter.upperBound!"">
  <#assign lowerBoundVar = "lower">
  <#assign upperBoundVar = "upper">

function ${setter.methodName}(${setter.parameterName}) {
var ${varName} = math.eval(${setter.parameterName});
  <#if lowerBound?has_content>
    var ${lowerBoundVar} = math.eval("${lowerBound}").toSI().toNumber();
  </#if>
  <#if upperBound?has_content>
    var ${upperBoundVar} = math.eval("${upperBound}").toSI().toNumber();
  </#if>

if (${varName} === undefined) {
  throw "Could not evaluate input for ${setter.parameterName}";
}

  <#if setter.dimension??>
//check dimension
var dim = math.matrix([${setter.dimension?join(", ")}]);
if (!math.deepEqual(${varName}.size(), dim)) {
  throw "Input has dimension " + ${varName}.size() + " but expected " + dim;
}

var array = [];
    <#assign dimensions = setter.dimension?size>
    <@forloop index = 0 dim = setter.dimension times = dimensions - 1>

  var ${elementName} = ${varName}.get([<#list 0..<dimensions as i>i${i}<#sep>,</#list>]);

      <@checkUnit unit=setter.unit!"" var=elementName/>
      var ${elementNumber} = ${elementName}.toSI().toNumber();
      <@checkRange var=elementNumber lowerBound=setter.lowerBound!"" upperBound=setter.upperBound!""/>
  array<@arrayindex n=dimensions/> = ${elementNumber};
    </@forloop>
Module.${setter.delegateMethodName}(array);
  <#else>
    <@checkUnit unit=setter.unit!"" var=varName/>
    var ${varNumber} = ${varName}.toSI().toNumber();
    <@checkRange var=varNumber lowerBound=setter.lowerBound!"" upperBound=setter.upperBound!""/>
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

<#macro checkUnit unit var>
  //check unit
  <#if unit?has_content>
  var expectedUnit = math.eval("${unit}");
  if (math.typeof(expectedUnit) !== math.typeof(${var}) || !expectedUnit.equalBase(${var})) {
    throw "Expected unit ${unit}";
  }
  </#if>
</#macro>

<#macro checkRange var lowerBound upperBound>
  //check range
  <#if lowerBound?has_content>
  if (math.smaller(${var}, ${lowerBoundVar})) {
    throw "Value " + ${var} + " out of range";
  }
  </#if>
  <#if upperBound?has_content>
  if (math.larger(${var}, ${upperBoundVar})) {
    throw "Value " + ${var} + " out of range";
  }
  </#if>
</#macro>