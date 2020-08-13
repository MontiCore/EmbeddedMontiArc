<#-- (c) https://github.com/MontiCore/monticore -->
<#assign networkDir = element.networkDir>
<#assign networkPrefix = element.networkPrefix>
<#assign numInputs = element.numInputs?c>
<#assign outputShape = "(1,"+ tc.join(element.outputShape, ",") + ")">
<#if mode == "ARCHITECTURE_DEFINITION">
            lastEpoch = 0
            for file in os.listdir("${networkDir}"):
                if "${networkPrefix}" in file and ".json" in file:
                    symbolFile = file

                if "${networkPrefix}" in file and ".param" in file:
                    epochStr = file.replace(".params", "").replace("${networkPrefix}", "")
                    epoch = int(epochStr)
                    if epoch >= lastEpoch:
                        lastEpoch = epoch
                        weightFile = file

<#if numInputs == "1">
            inputNames = ["data"]      
            zeroInputs = [nd.zeros(<#if element.element.inputTypes[0].dimensions[0] == 1 && tc.cutDimensionsInteger(element.element.inputTypes[0].dimensions)?size != 1>(${tc.join(element.element.inputTypes[0].dimensions?reverse, ",")})<#else>(1,${tc.join(element.element.inputTypes[0].dimensions?reverse, ",")})</#if>, ctx=mx_context[0])]
<#else>
            inputNames = []
            zeroInputs = []
<#list element.element.inputTypes as inType>
            inputNames.append("data" + str(${inType?index}))
            zeroInputs.append(nd.zeros(<#if inType.dimensions[0] == 1 && tc.cutDimensionsInteger(inType.dimensions)?size != 1>(${tc.join(tc.cutDimensionsInteger(inType.dimensions)?reverse, ",")})<#else>(1,${tc.join(tc.cutDimensionsInteger(inType.dimensions)?reverse, ",")})</#if>, ctx=mx_context[0]))
</#list>
</#if>
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                self.${element.name} = gluon.nn.SymbolBlock.imports("${networkDir}/" + symbolFile, inputNames, "${networkDir}/" + weightFile, ctx=mx_context)
            self.${element.name}out_shape = self.${element.name}(*zeroInputs).shape
            if self.${element.name}out_shape != ${outputShape}:
                outputSize=1
                for x in ${outputShape}: 
                    outputSize = outputSize * x  
                self.${element.name}fc_ = gluon.nn.Dense(units=outputSize, use_bias=False, flatten=False)

<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${tc.join(element.inputs, ",")})
        if self.${element.name}out_shape != ${outputShape}:
            ${element.name} = self.${element.name}fc(${element.name})
            ${element.name} = F.reshape(${element.name}, shape=${outputShape})
<#elseif mode == "PREDICTION_PARAMETER">
	    query_num_inputs.push_back(${numInputs});
</#if>
	