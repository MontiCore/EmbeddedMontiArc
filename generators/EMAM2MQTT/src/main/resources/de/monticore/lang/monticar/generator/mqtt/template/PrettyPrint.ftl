<#-- (c) https://github.com/MontiCore/monticore -->
<#--@formatter:off-->

Component name: ${model.getCompName()}

Ports: 

<#list model.getPortsDesc() as gen>
${gen}
</#list>
