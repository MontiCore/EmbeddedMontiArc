<#list tc.architecture.inputs as input>
<#if tc.getName(input)??>
    vector<float> ${tc.getName(input)} = CNNTranslator::translate(${input.name}<#if input.arrayAccess.isPresent()>[${input.arrayAccess.get().intValue.get()?c}]</#if>);
</#if>
</#list>
<#list tc.getLayerVariableMembers("1")?keys as member>
    vector<float> ${member}(${tc.join(tc.getLayerVariableMembers("1")[member], " * ")})
</#list>
<#list tc.architecture.outputs as output>
<#if tc.getName(output)??>
    vector<float> ${tc.getName(output)}(${tc.join(output.ioDeclaration.type.dimensions, " * ")});
</#if>
</#list>

<#list tc.architecture.streams as stream>
<#if stream.isTrainable()>
    _predictor_${stream?index}_.predict(${tc.join(tc.getStreamInputNames(stream), ", ")}, ${tc.join(tc.getStreamOutputNames(stream), ", ")});
<#else>
${tc.include(stream, "CPP_INLINE")}
</#if>
</#list>

<#list tc.architecture.unrolls as unroll>
<#list unroll.getBodiesForAllTimesteps() as body>
<#if body.isTrainable()>
    _predictor_${tc.architecture.streams?size + body?index}_.predict(${tc.join(tc.getStreamInputNames(body), ", ")}, ${tc.join(tc.getStreamOutputNames(body), ", ")});
<#else>
${tc.include(unroll, true, "CPP_INLINE")}
</#if>
</#list>
</#list>

<#list tc.architecture.outputs as output>
<#if tc.getName(output)??>
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
</#if>
</#list>
