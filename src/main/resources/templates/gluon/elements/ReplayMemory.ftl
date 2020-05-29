<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
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
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = ReplayMemory(replay_interval=${replayInterval}, replay_batch_size=${replayBatchSize}, replay_steps=${replaySteps},
                                                replay_gradient_steps=${replayGradientSteps}, store_prob=${replayMemoryStoreProb},
                                                max_stored_samples=${maxStoredSamples}, use_replay=${useReplay},
                                                query_net_dir="${queryNetDir}", 
                                                query_net_prefix="${queryNetPrefix}")
<#elseif mode == "FORWARD_FUNCTION">
<#if useReplay == "True" || useLocalAdaption == "true">
        ${element.name}full_, ind_${element.name} = self.${element.name}(*args)
<#else>
        ${element.name}full_, ind_${element.name} = self.${element.name}(${input})
</#if>
        ${element.name} = ${element.name}full_[0]
<#elseif mode == "PREDICTION_PARAMETER">
		use_local_adaption.push_back(${useLocalAdaption});
        dist_measure.push_back("${replayMemoryStoreDistMeasure}");
	    replay_k.push_back(${localAdaptionK});
        gradient_steps.push_back(${localAdaptionGradientSteps});
</#if>
	