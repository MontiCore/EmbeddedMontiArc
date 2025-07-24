<#-- (c) https://github.com/MontiCore/monticore -->
<#if mode == "ARCHITECTURE_DEFINITION">
            if data_mean:
                assert(data_std)
                self.input_normalization_${element.name} = ZScoreNormalization(data_mean=data_mean['${element.name}'],
                                                                               data_std=data_std['${element.name}'])
            else:
                self.input_normalization_${element.name} = NoNormalization()

<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.input_normalization_${element.name}(${element.name})
</#if>
