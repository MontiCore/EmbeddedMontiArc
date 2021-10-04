<#-- (c) https://github.com/MontiCore/monticore -->
<#if mode == "ARCHITECTURE_DEFINITION">
            self.repara_shape = (batch_size, <#list element.element.outputTypes as type>${type.dimensions[0]}</#list>)
</#if>
<#if mode == "FORWARD_FUNCTION">
        self.loss_param_val = []
        self.loss_param_val.append(${element.inputs[0]})
        self.loss_param_val.append(${element.inputs[1]})
        eps = F.random_normal(loc=0, scale=1, ctx=mx.cpu(), shape=self.repara_shape)
        reparametrization_ = ${element.inputs[0]} + F.exp(0.5 * ${element.inputs[1]}) * eps
</#if>
