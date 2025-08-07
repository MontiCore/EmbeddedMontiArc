<#-- (c) https://github.com/MontiCore/monticore -->
<#assign networkDir = element.networkDir>
<#assign networkPrefix = element.networkPrefix>
<#assign numInputs = element.numInputs?c>
<#assign outputShape = tc.join(element.outputShape, ",")>
<#assign trainable = element.trainable>
<#if mode == "ARCHITECTURE_DEFINITION">
        trainedFile = None
        for file in os.listdir("${networkDir}"):
            if "${networkPrefix}" in file and ".pt" in file:
                trainedFile = file

        <#if numInputs == "1">
            inputNames = ["data"]
            zeroInputs = [torch.zeros((1,${tc.join(element.element.inputTypes[0].dimensions, ",")}))]
        <#else>
            inputNames = []
            zeroInputs = []
            <#list element.element.inputTypes as inType>
            inputNames.append("data" + str(${inType?index}))
            zeroInputs.append(torch.zeros(1,${tc.join(tc.cutDimensionsInteger(inType.dimensions), ",")}))
            </#list>
        </#if>
        if trainedFile:
            self.${element.name} = torch.load("${networkDir}/" + trainedFile)
        else:
            raise FileNotFoundError("Model files were not found in '${networkDir}'.")
        <#if trainable == false>
            for parameter in self.${element.name}.parameters():
                parameter.requires_grad  = False
        </#if>

        outputSize=1
        for x in (${outputShape},):
            outputSize = outputSize * x
        self.${element.name}fc_ = nn.LazyLinear(out_features=outputSize, bias=False)

<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${tc.join(element.inputs, ",")})
        ${element.name} = self.${element.name}fc_(${element.name})
</#if>