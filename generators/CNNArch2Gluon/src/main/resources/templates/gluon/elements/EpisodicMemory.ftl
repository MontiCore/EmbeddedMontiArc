<#-- (c) https://github.com/MontiCore/monticore -->
<#assign replayInterval = element.replayInterval?c>
<#assign replayBatchSize = element.replayBatchSize?c>
<#assign replaySteps = element.replaySteps?c>
<#assign replayGradientSteps = element.replayGradientSteps?c>
<#assign memoryStoreProb = element.memoryStoreProb?c>
<#assign maxStoredSamples = element.maxStoredSamples?c>
<#assign memoryReplacementStrategy = element.memoryReplacementStrategy>
<#assign useReplay = element.useReplay?string("True", "False")>
<#assign useLocalAdaptationPy = element.useLocalAdaptation?string("True", "False")>
<#assign useLocalAdaptationCpp = element.useLocalAdaptation?string("true", "false")>
<#assign localAdaptationK = element.localAdaptationK?c>
<#assign localAdaptationGradientSteps = element.localAdaptationGradientSteps?c>
<#assign queryNetDir = element.queryNetDir>
<#assign queryNetPrefix = element.queryNetPrefix>
<#assign queryNetNumInputs = element.queryNetNumInputs>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = EpisodicMemory(replay_interval=${replayInterval}, replay_batch_size=${replayBatchSize}, replay_steps=${replaySteps},
                                                replay_gradient_steps=${replayGradientSteps}, store_prob=${memoryStoreProb},
                                                max_stored_samples=${maxStoredSamples}, memory_replacement_strategy="${memoryReplacementStrategy}", use_replay=${useReplay},
                                                use_local_adaptation=${useLocalAdaptationPy}, local_adaptation_gradient_steps=${localAdaptationGradientSteps}, k=${localAdaptationK},
                                                query_net_dir="${queryNetDir}/", 
                                                query_net_prefix="${queryNetPrefix}",
                                                query_net_num_inputs=${queryNetNumInputs})
<#elseif mode == "FORWARD_FUNCTION">
<#if element.inputs?size == 1>
<#if useReplay == "True" || useLocaladaptation == "true">
        ${element.name}full_, ind_${element.name} = self.${element.name}(*args)
<#else>
        ${element.name}full_, ind_${element.name} = self.${element.name}(${tc.join(element.inputs, ",")})
</#if>	
        ${element.name} = ${element.name}full_[0]
<#else>
<#if useReplay == "True" || useLocaladaptation == "true">
        ${element.name}full_, ind_${element.name} = self.${element.name}(*args)
<#else>
        ${element.name}full_, ind_${element.name} = self.${element.name}(${tc.join(element.inputs, ",")})
</#if>
        ${element.name} = ${element.name}full_
</#if>
<#elseif mode == "PREDICTION_PARAMETER">
		use_local_adaptation.push_back(${useLocalAdaptationCpp});
	    replay_k.push_back(${localAdaptationK});
        gradient_steps.push_back(${localAdaptationGradientSteps});
	    query_num_inputs.push_back(${queryNetNumInputs});
</#if>
	