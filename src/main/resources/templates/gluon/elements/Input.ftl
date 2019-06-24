<#assign mode = definition_mode.toString()>
<#if mode == "ARCHITECTURE_DEFINITION">
            if not data_mean is None:
                assert(not data_std is None)
                self.${element.name}_input_normalization = ZScoreNormalization(data_mean=data_mean, data_std=data_std)
            else:
                self.${element.name}_input_normalization = NoNormalization()

</#if>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}_input_normalization(${element.name})
</#if>