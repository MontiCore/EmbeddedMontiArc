<#-- (c) https://github.com/MontiCore/monticore -->
<#assign replayInterval = element.replayInterval?c>
<#assign replayBatchSize = element.replayBatchSize?c>
<#assign replaySteps = element.replaySteps?c>
<#assign replayGradientSteps = element.replayGradientSteps?c>
<#assign replayMemoryStoreProb = element.replayMemoryStoreProb?c>
<#assign maxStoredSamples = element.maxStoredSamples?c>
<#assign useReplay = element.useReplay?string("True", "False")>
<#assign useLocalAdaption = element.useLocalAdaption?string("true", "false")>
<#assign replayMemoryStoreDistMeasure = element.replayMemoryStoreDistMeasure>
<#assign localAdaptionK = element.localAdaptionK?c>
<#assign localAdaptionGradientSteps = element.localAdaptionGradientSteps?c>
<#assign queryNetDir = element.queryNetDir>
<#assign queryNetPrefix = element.queryNetPrefix>
<#assign queryNetNumInputs = element.queryNetNumInputs>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = EpisodicMemory(replay_interval=${replayInterval}, replay_batch_size=${replayBatchSize}, replay_steps=${replaySteps},
                                                replay_gradient_steps=${replayGradientSteps}, store_prob=${replayMemoryStoreProb},
                                                max_stored_samples=${maxStoredSamples}, use_replay=${useReplay},
                                                query_net_dir="${queryNetDir}", 
                                                query_net_prefix="${queryNetPrefix}",
                                                query_net_num_inputs=${queryNetNumInputs})
<#elseif mode == "FORWARD_FUNCTION">
<#if element.inputs?size == 1>
<#if useReplay == "True" || useLocalAdaption == "true">
        ${element.name}full_, ind_${element.name} = self.${element.name}(*args)
<#else>
        ${element.name}full_, ind_${element.name} = self.${element.name}(${tc.join(element.inputs, ",")})
</#if>	
        ${element.name} = ${element.name}full_[0]
<#else>
<#if useReplay == "True" || useLocalAdaption == "true">
        ${element.name}full_, ind_${element.name} = self.${element.name}(*args)
<#else>
        ${element.name}full_, ind_${element.name} = self.${element.name}(${tc.join(element.inputs, ",")})
</#if>
        ${element.name} = ${element.name}full_
</#if>
<#elseif mode == "PREDICTION_PARAMETER">
		use_local_adaption.push_back(${useLocalAdaption});
        dist_measure.push_back("${replayMemoryStoreDistMeasure}");
	    replay_k.push_back(${localAdaptionK});
        gradient_steps.push_back(${localAdaptionGradientSteps});
</#if>
	