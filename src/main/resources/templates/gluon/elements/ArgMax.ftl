<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        <#-- only passtrough method, argmax logic is applied in pythonExecute.ftl and CNNSupervisedTrainer.ftl -->
        ${element.name} = ${input}
</#if>
