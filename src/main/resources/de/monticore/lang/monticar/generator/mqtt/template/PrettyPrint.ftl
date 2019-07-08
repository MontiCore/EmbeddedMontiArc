<#--@formatter:off-->

Component name: ${model.getCompName()}

Ports: 

<#list model.getPortsDesc() as gen>
${gen}
</#list>