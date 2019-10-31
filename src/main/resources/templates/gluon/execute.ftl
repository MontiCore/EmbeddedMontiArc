<#list tc.architectureInputSymbols as input>
    vector<float> ${tc.getName(input)} = CNNTranslator::translate(${input.name}<#if input.arrayAccess.isPresent()>[${input.arrayAccess.get().intValue.get()?c}]</#if>);
</#list>
<#list tc.getLayerVariableMembers()?keys as member>
    vector<float> ${member}(${tc.join(tc.getLayerVariableMembers()[member], " * ")});
</#list>

<#list tc.architectureOutputSymbols as output>
    vector<float> ${tc.getName(output)}(${tc.join(output.ioDeclaration.type.dimensions, " * ")});
</#list>
<#list tc.architecture.constants as constant>
    vector<float> ${tc.getName(constant)}{${constant.intValue?c}};
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
${tc.include(networkInstruction.body, "CPP_INLINE")}
</#if>
</#if>
</#list>

<#list tc.architectureOutputSymbols as output>
<#assign shape = output.ioDeclaration.type.dimensions>
<#if shape?size == 1>
<#if (output.ioDeclaration.type.domain.isNaturalNumber() || output.ioDeclaration.type.domain.isWholeNumber())>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToIntCol(${tc.getName(output)}, std::vector<size_t> {${shape[0]?c}});
<#else>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToCol(${tc.getName(output)}, std::vector<size_t> {${shape[0]?c}});
</#if>
</#if>
<#if shape?size == 2>
<#if (output.ioDeclaration.type.domain.isNaturalNumber() || output.ioDeclaration.type.domain.isWholeNumber())>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToIntMat(${tc.getName(output)}, std::vector<size_t> {${shape[0]?c}, ${shape[1]?c}});
<#else>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToMat(${tc.getName(output)}, std::vector<size_t> {${shape[0]?c}, ${shape[1]?c}});
</#if>
</#if>
<#if shape?size == 3>
<#if (output.ioDeclaration.type.domain.isNaturalNumber() || output.ioDeclaration.type.domain.isWholeNumber())>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToIntCube(${tc.getName(output)}, std::vector<size_t> {${shape[0]?c}, ${shape[1]?c}, ${shape[2]?c}});
<#else>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToCube(${tc.getName(output)}, std::vector<size_t> {${shape[0]?c}, ${shape[1]?c}, ${shape[2]?c}});

</#if>
</#if>
</#list>
