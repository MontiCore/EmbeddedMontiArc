<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign num_embeddings = element.numEmbeddings?c>
<#assign beta = element.beta?c>
<#if mode == "ARCHITECTURE_DEFINITION">

            self.${element.name} = VectorQuantize(num_embeddings=${num_embeddings},
                                                  embedding_dim=<#list element.element.outputTypes as type>${type.dimensions[0]}</#list>,
                                                  input_shape=(<#list element.element.outputTypes as type>${tc.join(type.dimensions, ",")}</#list>),
                                                  total_feature_maps_size=int(<#list element.element.outputTypes as type>${tc.join(type.dimensions, "*")}</#list>))
            self.loss_ctx_dict = {"loss": "quantization_loss",
                                  "values": { "beta":${beta},}}


<#elseif mode == "FORWARD_FUNCTION">
        ${element.name}, ${element.name}_commit_loss, ${element.name}_codebook_loss = self.${element.name}(${input})
        loss_params.append(${element.name}_commit_loss)
        loss_params.append(${element.name}_codebook_loss)
        self.save_specific_params_list.append((self.${element.name}.name,self.${element.name}.collect_params()))
</#if>