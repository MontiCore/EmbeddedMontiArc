<#-- (c) https://github.com/MontiCore/monticore -->
<#assign inputMaxLength = element.max_length>
<#assign outputDim = element.output_dim>
<#if mode == "ARCHITECTURE_DEFINITION">
    import numpy as np
    position_enc = np.arange(${inputMaxLength}).reshape((-1, 1)) / (np.power(10000, (2. / ${outputDim}) * np.arange(${outputDim}).reshape((1, -1))))
    position_enc[:, 0::2] = np.sin(position_enc[:, 0::2])
    position_enc[:, 1::2] = np.cos(position_enc[:, 1::2])
    self.${element.name} =  position_enc
</#if>
