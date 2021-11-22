<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign num_embeddings = element.numEmbeddings?c>
<#assign beta = element.beta?c>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = VectorQuantize(num_embeddings=${num_embeddings},
                                                  embedding_dim=<#list element.element.outputTypes as type>${type.dimensions[0]}</#list>,
                                                  shape=(batch_size,<#list element.element.outputTypes as type>${type.dimensions[0]}</#list>),
                                                  total_feature_maps_size=int(batch_size<#list element.element.outputTypes as type>*${tc.join(type.dimensions, "*")}</#list>))
            self.loss_ctx_dict = {"loss": "quantization_loss",
                                  "values": { "beta":'${beta}',}}

<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
        loss_params = []
        loss_params.append(${input})
        loss_params.append(${element.name})
</#if>