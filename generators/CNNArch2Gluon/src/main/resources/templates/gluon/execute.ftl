<#-- (c) https://github.com/MontiCore/monticore -->
<#list tc.architectureInputSymbols as input>
    vector<float> ${tc.getName(input)} = CNNTranslator::translate(${input.name}<#if input.arrayAccess.isPresent()>[${input.arrayAccess.get().intValue.get()?c}]</#if>);
</#list>

<#if tc.architectureOutputSymbols?size gt 1>
<#assign outputName = tc.getNameWithoutIndex(tc.getName(tc.architectureOutputSymbols[0]))>
    vector<vector<float>> ${outputName}(${tc.architectureOutputSymbols?size});
    for (size_t i = 0; i < ${outputName}.size(); ++i) {
        ${outputName}[i].emplace_back(${tc.join(tc.architectureOutputSymbols[0].ioDeclaration.type.dimensions, " * ")});
    }
<#else>
<#list tc.architectureOutputSymbols as output>
    vector<float> ${tc.getName(output)}(${tc.join(tc.architectureOutputSymbols[0].ioDeclaration.type.dimensions, " * ")});<#sep>,
</#list>
</#if>

<#list tc.getLayerVariableMembers()?keys as member>
    vector<float> ${member}(${tc.join(tc.getLayerVariableMembers()[member], " * ")});
</#list>

<#list tc.architecture.constants as constant>
    vector<float> ${tc.getName(constant)}{${constant.intValue?c}};
</#list>

<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.isUnroll()>
    {
        int k = ${tc.getBeamSearchWidth(networkInstruction)};
<#list tc.getUnrollInputNames(networkInstruction, "1") as inputName>
<#if tc.getNameWithoutIndex(inputName) == tc.outputName>
        vector<pair<vector<vector<float>>, double>> sequences{make_pair(vector<vector<float>>{${inputName}}, 1.0)};
</#if>
</#list>

        for (size_t i = 1; i < ${tc.getBeamSearchMaxLength(networkInstruction)}; ++i) {
            vector<pair<vector<vector<float>>, double>> allCandidates;

            for (const pair<vector<vector<float>>, double>& p : sequences) {
                vector<vector<float>> seq = p.first;
                double score = p.second;

<#list tc.getUnrollInputNames(networkInstruction, "i") as inputName>
<#if tc.getNameWithoutIndex(inputName) == tc.outputName>
                ${inputName} = seq.back();
</#if>
</#list>
                _predictor_${networkInstruction?index}_.predict(${tc.join(tc.getUnrollInputNames(networkInstruction, "i"), ", ")}, ${tc.join(tc.getUnrollOutputNames(networkInstruction, "i"), ", ")});
<#list tc.getUnrollOutputNames(networkInstruction, "i") as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
<#if tc.endsWithArgmax(networkInstruction.body)>
                std::vector<float>::iterator maxElement = std::max_element(${outputName}.begin(), ${outputName}.end());
                ${outputName} = std::vector<float>{static_cast<float>(std::distance(${outputName}.begin(), maxElement))};
</#if>
                vector<float> out = ${outputName};
</#if>
</#list>

                vector<pair<int, float>> topk;
                for (size_t i = 0; i < out.size(); ++i) {
                    topk.emplace_back(i, out[i]);
                }

                sort(topk.begin(), topk.end(), [] (const pair<int, float>& p1, const pair<int, float>& p2) {
                    return p1.second > p2.second;
                });
                topk = vector<pair<int, float>>(topk.begin(), topk.begin() + std::min<int>(k, topk.size()));

                for (const pair<int, float>& pair : topk) {
                    vector<vector<float>> currentSeq = seq;
                    currentSeq.push_back(vector<float>{(float) pair.first});
                    allCandidates.emplace_back(currentSeq, score * pair.second);
                }
            }

            sort(allCandidates.begin(), allCandidates.end(), [] (const pair<vector<vector<float>>, double>& p1, const pair<vector<vector<float>>, double>& p2) {
                return p1.second > p2.second;
            });
            sequences = vector<pair<vector<vector<float>>, double>>(allCandidates.begin(), allCandidates.begin() + std::min<int>(k, allCandidates.size()));
        }

        for (size_t i = 1; i < ${tc.getBeamSearchMaxLength(networkInstruction)}; ++i) {
<#list tc.getUnrollOutputNames(networkInstruction, "i") as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
            ${outputName} = sequences[0].first[i];
</#if>
</#list>
        }
    }
<#else>
    _predictor_${networkInstruction?index}_.predict(${tc.join(tc.getStreamInputNames(networkInstruction.body, true), ", ")}, ${tc.join(tc.getStreamOutputNames(networkInstruction.body, true), ", ")});
<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
<#if tc.endsWithArgmax(networkInstruction.body)>
    std::vector<float>::iterator maxElement = std::max_element(${outputName}.begin(), ${outputName}.end());
    ${outputName} = std::vector<float>{static_cast<float>(std::distance(${outputName}.begin(), maxElement))};
</#if>
</#if>
</#list>
</#if>

</#list>
<#list tc.architectureOutputSymbols as output>
<#assign shape = output.ioDeclaration.type.dimensions>
<#if shape?size == 1>
<#if (output.ioDeclaration.type.domain.isNaturalNumber() || output.ioDeclaration.type.domain.isWholeNumber())>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToIntCol(${tc.getNameAsArray(tc.getName(output))}, std::vector<size_t> {${shape[0]?c}});
<#else>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToCol(${tc.getNameAsArray(tc.getName(output))}, std::vector<size_t> {${shape[0]?c}});
</#if>
</#if>
<#if shape?size == 2>
<#if (output.ioDeclaration.type.domain.isNaturalNumber() || output.ioDeclaration.type.domain.isWholeNumber())>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToIntMat(${tc.getNameAsArray(tc.getName(output))}, std::vector<size_t> {${shape[0]?c}, ${shape[1]?c}});
<#else>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToMat(${tc.getNameAsArray(tc.getName(output))}, std::vector<size_t> {${shape[0]?c}, ${shape[1]?c}});
</#if>
</#if>
<#if shape?size == 3>
<#if (output.ioDeclaration.type.domain.isNaturalNumber() || output.ioDeclaration.type.domain.isWholeNumber())>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToIntCube(${tc.getNameAsArray(tc.getName(output))}, std::vector<size_t> {${shape[0]?c}, ${shape[1]?c}, ${shape[2]?c}});
<#else>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToCube(${tc.getNameAsArray(tc.getName(output))}, std::vector<size_t> {${shape[0]?c}, ${shape[1]?c}, ${shape[2]?c}});
</#if>
</#if>
<#if shape?size == 4>
<#if (output.ioDeclaration.type.domain.isNaturalNumber() || output.ioDeclaration.type.domain.isWholeNumber())>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToIntCube(${tc.getNameAsArray(tc.getName(output))}, std::vector<size_t> {${shape[1]?c}, ${shape[2]?c}, ${shape[3]?c}});
<#else>
    ${output.name}<#if output.arrayAccess.isPresent()>[${output.arrayAccess.get().intValue.get()?c}]</#if> = CNNTranslator::translateToCube(${tc.getNameAsArray(tc.getName(output))}, std::vector<size_t> {${shape[1]?c}, ${shape[2]?c}, ${shape[3]?c}});
</#if>
</#if>
</#list>
