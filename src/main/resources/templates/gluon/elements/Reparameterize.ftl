<#-- (c) https://github.com/MontiCore/monticore -->
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Reparameterize(shape=(<#list element.element.outputTypes as type>${type.dimensions[0]}</#list>,), pdf='${element.pdf}',)
            self.loss_ctx_dict = { "loss":"kl_div_loss",<#-- for later if any other metrics than KL are known-->
                                    "values": { "pdf":'${element.pdf}',
                                                "shape": (<#list element.element.outputTypes as type>${type.dimensions[0]}</#list>,)}}
</#if>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}([${tc.join(element.inputs, ", ")}])
        <#list element.inputs as input>
        loss_params.append(${input})
        </#list>
</#if>

<#--
#self.loss_param_val = []
#self.loss_param_val.append(${element.inputs[0]})
#self.loss_param_val.append(${element.inputs[1]})
#eps = F.random_normal(loc=0, scale=1, ctx=mx.cpu(), shape=self.repara_shape)
#${element.name} = ${element.inputs[0]} + F.exp(0.5 * ${element.inputs[1]}) * eps -->