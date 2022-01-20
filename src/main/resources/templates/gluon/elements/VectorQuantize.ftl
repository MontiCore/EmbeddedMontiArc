<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign num_embeddings = element.numEmbeddings?c>
<#assign beta = element.beta?c>
<#if mode == "ARCHITECTURE_DEFINITION">

            self.${element.name} = VectorQuantize(batch_size=batch_size,
                                                  num_embeddings=${num_embeddings},
                                                  embedding_dim=<#list element.element.outputTypes as type>${type.dimensions[0]}</#list>,
                                                  input_shape=(batch_size,<#list element.element.outputTypes as type>${tc.join(type.dimensions, ",")}</#list>),
                                                  total_feature_maps_size=int(batch_size*<#list element.element.outputTypes as type>${tc.join(type.dimensions, "*")}</#list>))
            self.loss_ctx_dict = {"loss": "quantization_loss",
                                  "values": { "beta":${beta},}}


<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
        loss_params.append(${input})
        loss_params.append(${element.name})
        self.save_specific_params_list.append((self.${element.name}.name,self.${element.name}.collect_params()))
</#if>