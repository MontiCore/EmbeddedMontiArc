<#list tc.architecture.outputs as output>
    vector<float> CNN_${tc.getName(output)}(<#list output.definition.type.dimensions as dim>${dim?c}<#sep>*</#list>);
</#list>

<#list tc.architecture.streams as stream>
<#if stream.isNetwork()>
    _predictor_${stream?index}_.predict(<#list stream.getFirstAtomicElements() as input>CNNTranslator::translate(${input.name}<#if input.arrayAccess.isPresent()>[${input.arrayAccess.get().intValue.get()?c}]</#if>),
                </#list><#list stream.getLastAtomicElements() as output>CNN_${tc.getName(output)}<#sep>,
                </#list>);
</#if>
</#list>

<#list tc.architecture.outputs as output>
<#assign shape = output.definition.type.dimensions>
<#if shape?size == 1>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToCol(CNN_${tc.getName(output)}, std::vector<size_t> {${shape[0]?c}});
</#if>
<#if shape?size == 2>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToMat(CNN_${tc.getName(output)}, std::vector<size_t> {${shape[0]?c}, ${shape[1]?c}});
</#if>
<#if shape?size == 3>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToCube(CNN_${tc.getName(output)}, std::vector<size_t> {${shape[0]?c}, ${shape[1]?c}, ${shape[2]?c}});
</#if>
</#list>
