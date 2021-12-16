<#-- (c) https://github.com/MontiCore/monticore -->
<#assign networkDir = element.networkDir>
<#assign networkPrefix = element.networkPrefix>
<#assign numInputs = element.numInputs?c>
<#assign outputShape = tc.join(element.outputShape, ",")>
<#if mode == "ARCHITECTURE_DEFINITION">
            lastEpoch = 0
            symbolFile = None
            weightFile = None
            onnxFile = None
            for file in os.listdir("${networkDir}"):
                if "${networkPrefix}" in file and ".json" in file:
                    symbolFile = file

                if "${networkPrefix}" in file and ".param" in file:
                    epochStr = file.replace(".params", "").replace("${networkPrefix}", "")
                    epoch = int(epochStr)
                    if epoch >= lastEpoch:
                        lastEpoch = epoch
                        weightFile = file

                if "${networkPrefix}" in file and ".onnx" in file:
                    onnxFile = file

<#if numInputs == "1">
            inputNames = ["data"]      
            zeroInputs = [nd.zeros((1,${tc.join(element.element.inputTypes[0].dimensions, ",")}), ctx=mx_context[0])]
<#else>
            inputNames = []
            zeroInputs = []
<#list element.element.inputTypes as inType>
            inputNames.append("data" + str(${inType?index}))
            zeroInputs.append(nd.zeros((1,${tc.join(tc.cutDimensionsInteger(inType.dimensions), ",")}), ctx=mx_context[0]))
</#list>
</#if>
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                if symbolFile and weightFile:
                    self.${element.name} = gluon.nn.SymbolBlock.imports("${networkDir}/" + symbolFile, inputNames, "${networkDir}/" + weightFile, ctx=mx_context)
                elif onnxFile:
                    from mxnet.contrib import onnx as onnx_mxnet
                    <#--  Works but how inputNames are defined here?  -->
                    <#--  self.${element.name} = onnx_mxnet.import_to_gluon("${networkDir}/" + onnxFile, ctx=mx_context)  -->
                    sym, arg_params, aux_params = onnx_mxnet.import_model("${networkDir}/" + onnxFile)
                    inputSymVars = [mx.sym.var(inputName) for inputName in inputNames]
                    self.${element.name} = gluon.nn.SymbolBlock(outputs=sym, inputs=inputSymVars)
                    net_params = self.${element.name}.collect_params()
                    for param in arg_params:
                        if param in net_params:
                            net_params[param]._load_init(arg_params[param], ctx=mx_context)
                    for param in aux_params:
                        if param in net_params:
                            net_params[param]._load_init(aux_params[param], ctx=mx_context)
                else:
                    raise FileNotFoundError("Model files were not found in '${networkDir}'.")

            self.${element.name}out_shape = self.${element.name}(*zeroInputs).shape
            if self.${element.name}out_shape != (1,${outputShape}):
                outputSize=1
                for x in (${outputShape},): 
                    outputSize = outputSize * x  
                self.${element.name}fc_ = gluon.nn.Dense(units=outputSize, use_bias=False, flatten=False)

<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${tc.join(element.inputs, ",")})
        if self.${element.name}out_shape != (1,${outputShape}):
            ${element.name} = self.${element.name}fc_(${element.name})
            ${element.name} = F.reshape(${element.name}, shape=(-1,${outputShape}))
</#if>
	