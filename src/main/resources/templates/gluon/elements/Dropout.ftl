<#-- (c) https://github.com/MontiCore/monticore -->
<#assign rate = element.p?c>
<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = gluon.nn.Dropout(rate=${rate})
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
