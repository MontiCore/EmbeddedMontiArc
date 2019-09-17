<#list tc.architectureInputSymbols as input>
    vector<float> ${tc.getName(input)} = CNNTranslator::translate(${input.name}<#if input.arrayAccess.isPresent()>[${input.arrayAccess.get().intValue.get()?c}]</#if>);
</#list>
<#list tc.getLayerVariableMembers("1")?keys as member>
    vector<float> ${member}(${tc.join(tc.getLayerVariableMembers("1")[member][0], " * ")});
</#list>

<#list tc.architectureOutputSymbols as output>
    vector<float> ${tc.getName(output)}(${tc.join(output.ioDeclaration.type.dimensions, " * ")});
</#list>

<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.isUnroll()>
<#list networkInstruction.toUnrollInstruction().resolvedBodies as resolvedBody>
    _predictor_${networkInstruction?index}_.predict(${tc.join(tc.getStreamInputNames(networkInstruction.body, resolvedBody), ", ")}, ${tc.join(tc.getStreamOutputNames(networkInstruction.body, resolvedBody), ", ")});
</#list>
<#else>
<#if networkInstruction.body.isTrainable()>
    _predictor_${networkInstruction?index}_.predict(${tc.join(tc.getStreamInputNames(networkInstruction.body), ", ")}, ${tc.join(tc.getStreamOutputNames(networkInstruction.body), ", ")});
<#else>
<#-- ${tc.include(networkInstruction.body, "CPP_INLINE")}; -->
</#if>
</#if>
</#list>

<#list tc.architectureOutputSymbols as output>
<#assign shape = output.ioDeclaration.type.dimensions>
<#if shape?size == 1>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToCol(${tc.getName(output)}, std::vector<size_t> {${shape[0]?c}});
</#if>
<#if shape?size == 2>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToMat(${tc.getName(output)}, std::vector<size_t> {${shape[0]?c}, ${shape[1]?c}});
</#if>
<#if shape?size == 3>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToCube(${tc.getName(output)}, std::vector<size_t> {${shape[0]?c}, ${shape[1]?c}, ${shape[2]?c}});
</#if>
</#list>
