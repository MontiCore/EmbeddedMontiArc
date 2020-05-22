<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign subKeySize = element.subKeySize?c>
<#assign querySize = "[" + tc.join(element.querySize, ",") + "]">
<#assign queryAct = element.queryAct>
<#assign replayMemoryStoreDistMeasure = element.replayMemoryStoreDistMeasure>
<#assign k = element.k?c>
<#assign numHeads = element.numHeads?c>
<#if element.valueShape[0] == -1>       
<#assign valueShape = "(" + tc.join(element.element.getInputTypes()[0].dimensions, ",") + ")">
<#else>
<#assign valueShape = "(" + tc.join(element.valueShape, ",") + ")">
</#if>
<#assign useReplay = element.useReplay?string("True", "False")>
<#assign replayInterval = element.replayInterval?c>
<#assign replayBatchSize = element.replayBatchSize?c>
<#assign replaySteps = element.replaySteps?c>
<#assign replayGradientSteps = element.replayGradientSteps?c>
<#assign replayMemoryStoreProb = element.replayMemoryStoreProb?c>
<#assign useLocalAdaption = element.useLocalAdaption?string("true", "false")>
<#assign localAdaptionK = element.localAdaptionK?c>
<#assign localAdaptionGradientSteps = element.localAdaptionGradientSteps?c>
<#assign localAdaptionMemoryStoreDistMeasure = element.localAdaptionMemoryStoreDistMeasure>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Memory(sub_key_size=${subKeySize}, query_size=${querySize}, query_act="${queryAct}", 
                                          dist_measure="${replayMemoryStoreDistMeasure}", k=${k}, num_heads=${numHeads}, 
                                          use_replay=${useReplay}, replay_interval=${replayInterval}, 
                                          replay_batch_size=${replayBatchSize}, replay_steps=${replaySteps},
                                          replay_gradient_steps=${replayGradientSteps}, store_prob=${replayMemoryStoreProb}, 
                                          value_shape=${valueShape})
<#elseif mode == "FORWARD_FUNCTION">
<#if useReplay == "True" || useLocalAdaption == "true">
        ${element.name}, ind_${element.name} = self.${element.name}(x)
<#else>
        ${element.name}, ind_${element.name} = self.${element.name}(${input})
</#if>
<#elseif mode == "PREDICTION_PARAMETER">
		use_local_adaption.push_back(${useLocalAdaption});
        dist_measure.push_back("${localAdaptionMemoryStoreDistMeasure}");
	    replay_k.push_back(${localAdaptionK});
        gradient_steps.push_back(${localAdaptionGradientSteps});
        num_heads.push_back(${numHeads});
</#if>
