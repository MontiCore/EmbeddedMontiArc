<#--
constant declarations: (pattern: portName -> z3 constant name)
    port -> port
    port[1] -> port*1
    port[?] -> port*?
    subComp.port -> subComp.port
    subComp[1].port -> subComp*1.port
    subComp[1].port[1] -> subComp*1.port*1
  from events:
    port::connect{ } -> port*connect (not for ports in subcomponents)
    port::value([0,0,0]) -> port**2, port**1, port
-->



(echo "${PREAMBLE}:value=${symTab.getFullName()};")


  <#-- returns translated name 'port[n]' to 'port*n' -->
<#function getPortName(name)>
  <#return name?replace("\\[(.*?)\\]","*$1", "r")>
</#function>


  <#-- returns value 'n' from events like port::value(n) -->
<#function getAssertValue(portValue)>
    <#if portValue.isVariable() == true>
        <#return portValue.getVariableValue()>
    <#elseif portValue.isLogic() == true>
        <#return portValue.isLogicValue()?c>
    <#elseif portValue.isNumber() == true>
        <#return portValue.getNumberValue()>
    </#if>
    <#return 0>
</#function>


  <#-- creates new constant(s) based on a port (array), if dynamic: create also constants 'port*?' and 'port*connect' -->
<#macro printPortOrPortArray port name>
    <#assign isPortOfSubComp = name?contains(".")>
    <#if instanceOf(port, EMADynamicPortArraySymbol)>
        <#if port.isDynamic()>
            <@printPort port getPortName(name+"[?]")/>
            <#if isPortOfSubComp == false>
                <@printBooleanPort getPortName(name+"[connect]")/>
            </#if>
        </#if>
        <#list 1..port.isDynamic()?then(port.getNonDynamicDimension(), port.getDimension()) as i>
            <#assign mergedPortName = getPortName(name+"["+i+"]")>
            <@printPort port mergedPortName/>
        </#list>
    <#else>
        <@printPort port name/>
        <#if port.getAstNode().isPresent() && port.getAstNode().get().isDynamic()>
            <#if port.getName()?ends_with("[1]") && isPortOfSubComp == false>
                <@printPort port getPortName(port.getName()?replace("[1]","[?]"))/>
                <@printBooleanPort getPortName(port.getName()?replace("[1]","[connect]"))/>
            </#if>
        </#if>
    </#if>
</#macro>


<#macro printBooleanPort name>
(declare-const ${name} Bool)
</#macro>


  <#-- prints '(declare-const NAME TYPE)' for a given port
       if range for port defined: create assertions for upper and lower borders;
       if port is constant: create assertion that fixes the value to the constant's value -->
<#macro printPort port name>
(declare-const ${name} <#t>
                 <#switch port.getTypeReference().getName() >
                                   <#case "B">Bool<#break>
                                   <#case "UnitNumberResolution"><#case "Q">Real<#break>
                                   <#case "Z">Int<#break>
                                   <#case "CommonMatrixType">(Matrix Real)<#break>
                                   <#default>(Object Bool)<#break>
                 </#switch>)
                <#if port.getTypeReference().getName() != "CommonMatrixType">
             <#if port.isConstant() == true>
(assert (= ${name} ${port.getConstantValue().get().getValueAsString()}))
             </#if>
             <#if port.getAstNode().isPresent()><#if port.getAstNode().get().getType().getRangeOpt().isPresent()><#if port.getAstNode().get().getType().getRangeOpt().get().getMax().getNumber().isPresent()>
(assert (<= ${name} ${port.getAstNode().get().getType().getRangeOpt().get().getMax().getNumber().get()}))
             </#if></#if></#if>
             <#if port.getAstNode().isPresent()><#if port.getAstNode().get().getType().getRangeOpt().isPresent()><#if port.getAstNode().get().getType().getRangeOpt().get().getMin().getNumber().isPresent()>
(assert (>= ${name} ${port.getAstNode().get().getType().getRangeOpt().get().getMin().getNumber().get()}))
             </#if></#if></#if></#if>
</#macro>


  <#-- create new ports for port values that occur in sequences, like in 'port::value([0,0,0])'' -->
<#macro printPortSequences condition lineNr>
    <#if instanceOf(condition, EventLogicalOperationExpressionSymbol)>
                <#if condition.getOperator() == "&&">
                    <@printPortSequences condition.getLeftExpression() lineNr/>
                    <@printPortSequences condition.getRightExpression() lineNr/>
                </#if>
    <#elseif instanceOf(condition, EventBracketExpressionSymbol)>
        <@printPortSequences condition.getInnerExpression() lineNr/>
    <#elseif instanceOf(condition, EventPortExpressionConnectSymbol) == false && instanceOf(condition, EventPortExpressionFreeSymbol) == false>
        <#assign portValue = condition.getPortValue()>
        <#if instanceOf(portValue, PortValuesArraySymbol) && portValue.size() gt 1>
            <#list 2..portValue.size() as i>
                <#assign name = getPortName(condition.getName())+"**"+(i-1)>
                <#if PortInSequenceAlreadyDefined(DefinedPortsInSequences, name) == false>
                    <#list symTab.getPortsList() as port>
                        <#if port.getName() == condition.getName()?replace("\\[(.*)\\]","", "r")>
                            <@printPort port name/>
                        </#if>
                    </#list>
                </#if>
            </#list>
        </#if>
    </#if>
</#macro>


  <#-- creates assertions for events, like 'name::value(> VALUE)' -> '(assert (> NAME VALUE))' -->
<#macro printCondition operator name value>
    <#if instanceOf(value, PortValueInputSymbol)>
        <#if operator == "!=" && value??>
(not (= ${name} ${getAssertValue(value)}))<#t>
        <#else>
(${operator} ${name} ${getAssertValue(value)})<#t>
        </#if>
    <#elseif instanceOf(value, PortValueCompareSymbol)>
        <#assign innerPortValue = value.getCompareValue()>
        <@printCondition value.getOperator() name innerPortValue/>
    </#if>
</#macro>


  <#-- traverses the tree (recursively, if needed) that holds conditions of an event -->
<#macro printReconfigurationConditions condition>
    <#assign name = getPortName(condition.getName())>
    <#assign operator = "=">
    <#if instanceOf(condition, EventBracketExpressionSymbol)>
<@printReconfigurationConditions condition.getInnerExpression()/><#t>
    <#elseif instanceOf(condition, EventLogicalOperationExpressionSymbol)>
        <#if condition.getOperator() == "&&">
(and <#t><@printReconfigurationConditions condition.getLeftExpression()/> <@printReconfigurationConditions condition.getRightExpression()/><#t>)<#t>
        <#elseif condition.getOperator() == "||">
(or <#t><@printReconfigurationConditions condition.getLeftExpression()/> <@printReconfigurationConditions condition.getRightExpression()/><#t>)<#t>
        <#elseif condition.getOperator() == "!">
            <#if condition.getRightExpression()??>
(not <@printReconfigurationConditions condition.getRightExpression()/>)<#t>
            </#if>
        </#if>
    <#elseif instanceOf(condition, EventPortExpressionValueSymbol)>
        <#assign portValue = condition.getPortValue()>
        <#if instanceOf(portValue, PortValuesArraySymbol)>
            <#assign size = portValue.size()>
            <#assign name = getPortName(condition.getName())>
            (and <#list 0..size-1 as i><#t>
                <#if i == size-1>
                    <@printCondition operator name portValue.getForIndex(i)/><#t>
                <#else>
                    <@printCondition operator name+"**"+(size-(i+1)) portValue.getForIndex(i)/><#t>
                </#if>
            </#list>)<#t>
        <#else>
            <@printCondition operator name portValue/>
        </#if>
    <#elseif instanceOf(condition, EventPortExpressionConnectSymbol)>
        (= ${name}*connect true)<#t>
    <#elseif instanceOf(condition, EventPortExpressionFreeSymbol)>
        (or true false)<#t><#-- don't handle value::free events -->
    <#elseif instanceOf(condition, EventBooleanExpressionSymbol)>
(= ${name} ${condition.getBooleanValue()?c})<#t>
    </#if>
</#macro>


<#-- ENTRY POINT OF THE GENERATION -->


  <#-- create datatypes for unused and unknown port types -->
(declare-datatypes (T1) ((Matrix (T1))))
(declare-datatypes (T2) ((Object (T2))))


  <#-- traverse port list -->
<#list symTab.getPortsList() as port>
    <@printPortOrPortArray port getPortName(port.getName())/>
</#list>


  <#-- traverse port list of any subcomponent -->
<#list symTab.getSubComponents() as sub>
    <#assign subCompName = "">
    <#if sub.getInstanceInformation().isPresent() == true>
        <#assign subCompName = sub.getInstanceInformation().get().getCompName()+".">
    </#if>
    <#if sub.isArray()>
        <#list sub.isDynamic()?then(0,1)..sub.isDynamic()?then(sub.getNonDynamicDimension(),sub.getDimension()) as i>
            <#if sub.getInstanceInformation().isPresent() == true>
                <#assign index = (i==0)?then("?",i)>
                <#assign subCompName = sub.getInstanceInformation().get().getCompName()+"*"+index+".">
            </#if>
            <#list sub.getComponentType().getPortsList() as port>
                <#if port.isIncoming() == true && port.isConstant() == false>
                    <@printPortOrPortArray port subCompName+getPortName(port.getName())/>
                </#if>
            </#list>
        </#list>
    <#else>
        <#if sub.getInstanceInformation().isPresent() == true>
            <#assign subCompName = sub.getInstanceInformation().get().getCompName()+".">
        </#if>
        <#list sub.getComponentType().getPortsList() as port>
            <#if port.isIncoming() == true && port.isConstant() == false>
                <@printPortOrPortArray port subCompName+getPortName(port.getName())/>
            </#if>
        </#list>
    </#if>
</#list>


  <#-- perform initial check for satisfiability (invalid ranges might cause contradictions)
       if unsat: any following check will be unsat, too. Therefore, the consistency check should then be cancelled -->
(check-sat)
(echo "${INITIALLYSATISFIABLE}:")


  <#-- append constants that occur in sequences -->
<#list symTab.getEventHandlers() as event>
    <#assign lineNr = 0>
    <#if event.getAstNode().isPresent()>
        <#assign lineNr = event.getAstNode().get().get_SourcePositionStart().getLine()>
    </#if>
    <@printPortSequences event.getCondition() lineNr />
</#list>

  <#-- add checks for events: single and pairwise -->
<#list symTab.getEventHandlers() as event>
    <#assign condition = event.getCondition()>
(push)
    (assert <@printReconfigurationConditions condition/>)
    (check-sat)
    <#assign connects = event.getConnectors()>
    <#if event.getAstNode().isPresent()>
(echo "${SINGLECONDNOTSAT}:line=${event.getAstNode().get().get_SourcePositionStart().getLine()};")
    </#if>
(echo "${clashCheck(initialConnects, connects)}")
  <#-- check if two events are pairwise satisfiable -->
(pop)
    <#list symTab.getEventHandlers() as event2>
        <#if event?index < event2?index>
    (push)
             <#assign condition2 = event2.getCondition()>
        (assert <@printReconfigurationConditions condition/>)
        (assert <@printReconfigurationConditions condition2/>)
        (check-sat)
             <#assign connects = event.getConnectors()>
             <#assign connects2 = event2.getConnectors()>
        (echo "${clashCheck(initialConnects, connects, connects2)}")
    (pop)
    (push)
            (assert (not (= <@printReconfigurationConditions condition/> <@printReconfigurationConditions condition2/>)))
            (check-sat)
            (echo "${EQUALCONDITIONS}:{${event.getAstNode().get().get_SourcePositionStart().getLine()},${event2.getAstNode().get().get_SourcePositionStart().getLine()}};")
    (pop)
        </#if>
    </#list>
    <#list event.getConnectors() as connector>
            <#if connector.isConstant() == true>
      <#-- check if connectors like 'connect 500 -> port' do not violate the range of port -->
(push)
                <#assign targetName = getPortName(connector.getTarget()?replace("\\[(.*?)\\].","*$1.", "r"))>
    (assert (= ${targetName} ${connector.getSource()}))
    (check-sat)
                <#if connector.getAstNode().isPresent()>
    (echo "${INVALIDBOUNDARY}:line=${connector.getAstNode().get().get_SourcePositionStart().getLine()};")
                </#if>
(pop)
            </#if>
        </#list>
</#list>