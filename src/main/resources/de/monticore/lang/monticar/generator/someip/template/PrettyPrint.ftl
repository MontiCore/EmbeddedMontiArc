<#--@formatter:off-->

Component name: ${model.getCompName()}

Ports:

<#list model.getPorts() as gen>
${gen}
</#list>
