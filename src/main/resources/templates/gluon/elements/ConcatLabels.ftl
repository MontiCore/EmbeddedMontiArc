<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name}preproc = gluon.nn.Flatten()
            self.${element.name}postproc = gluon.nn.Dense(units=<#list element.element.outputTypes as type>${tc.join(type.dimensions, "*")}</#list>, use_bias=False, flatten=True)

</#if>
<#if mode == "FORWARD_FUNCTION">
        ${element.name}1 = self.${element.name}preproc(${input})
        ${element.name}2 = F.concat(${element.name}1,labels, dim=1)
        ${element.name}3 = self.${element.name}postproc(${element.name}2)
        ${element.name} = F.reshape(${element.name}3, shape=(0,<#list element.element.outputTypes as type>${tc.join(type.dimensions, ",")}</#list>) )
</#if>