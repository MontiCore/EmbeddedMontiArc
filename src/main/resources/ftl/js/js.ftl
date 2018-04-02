<#ftl strip_whitespace=true>
var Module = {
  'print': function(text) { console.log('stdout: ' + text) },
  'printErr': function(text) { console.log('stderr: ' + text) },
  onRuntimeInitialized: function() {Module.init();}
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
function ${setter.methodName}(${setter.parameterName}) {
var value = math.eval(${setter.parameterName});

if (value === undefined) {
  throw "Could not evaluate input for ${setter.parameterName}";
}

  <#if setter.dimension??>
//check dimension
var dim = math.matrix([ ${setter.dimension?join(",")} ]);
if (!math.deepEqual(value.size(), dim)) {
  throw "Input has dimension " + value.size() + " but expected " + dim;
}

var array = [];
    <#assign dimensions = setter.dimension?size>
    <@forloop index = 0 dim = setter.dimension times = dimensions - 1>
      <#assign varName = "e">
  var ${varName} = value.get([<#list 0..<dimensions as i>i${i}<#sep>,</#list>]);

      <@checkUnit unit=setter.unit var=varName/>
      <@checkRange var=varName lowerBound=setter.lowerBound upperBound=setter.upperBound/>
  array<@arrayindex n=dimensions/> = e.toSI().toNumber();
    </@forloop>
Module.${setter.delegateMethodName}(array);
  <#else>
    <#assign varName = "value">
var ${varName} = math.eval(${setter.parameterName});

if (c === undefined) {
    throw "Could not evaluate input for ${setter.parameterName}";
}

    <@checkUnit unit=setter.unit var=varName/>
    <@checkRange var=varName lowerBound=setter.lowerBound upperBound=setter.upperBound/>
Module.${setter.delegateMethodName}(value.toSI().toNumber());
  </#if>
}
</#list>

<#macro forloop index dim times>
  for (var i${index} = 0; i${index} < ${dim[index]}; i${index}++) {
  <#if times gt 0>
    array<@arrayindex n=index/> = [];
    <@forloop index=index+1 dim=dim times=times-1></@forloop>
  <#else>
    <#nested>
  </#if>
  }
</#macro>

<#macro arrayindex n>
  <#list 0..n as i>[i${i}]</#list>
</#macro>

<#macro checkUnit unit var>
  //check unit
  var expectedUnit = math.eval("${unit}");
  if (math.typeof(expectedUnit) !== math.typeof(${var}) || !expectedUnit.equalBase(${var})) {
    throw "Expected unit ${unit}";
  }
</#macro>

<#macro checkRange var lowerBound upperBound>
  //check range
  <#if lowerBound??>
  if (math.smaller(${var}, math.eval("${lowerBound}"))) {
    throw "Value " + ${var} + " out of range";
  }
  </#if>
  <#if upperBound??>
  if (math.larger(${var}, math.eval("${upperBound}"))) {
    throw "Value " + ${var} + " out of range";
  }
  </#if>
</#macro>