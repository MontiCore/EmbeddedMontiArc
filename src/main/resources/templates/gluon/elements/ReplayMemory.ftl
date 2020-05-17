<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign replayInterval = element.replayInterval?c>
<#assign replayBatchSize = element.replayBatchSize?c>
<#assign replaySteps = element.replaySteps?c>
<#assign replayGradientStepsTraining = element.replayGradientStepsTraining?c>
<#assign storeProb = element.storeProb?c>
<#assign maxStoredSamples = element.maxStoredSamples?c>
<#assign replayK = element.replayK?c>
<#assign replayGradientStepsPrediction = element.replayGradientStepsPrediction?c>
<#assign queryNetDir = element.queryNetDir>
<#assign queryNetPrefix = element.queryNetPrefix>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = ReplayMemory(replay_interval=${replayInterval}, replay_batch_size=${replayBatchSize}, replay_steps=${replaySteps},
                                                replay_gradient_steps=${replayGradientStepsTraining}, store_prob =${storeProb}, max_stored_samples=${maxStoredSamples},
                                                query_net_dir="${queryNetDir}", 
                                                query_net_prefix="${queryNetPrefix}")
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(x)
<#elseif mode == "PREDICTION_PARAMETER">
	    replay_k.push_back(${replayK});
        gradient_steps.push_back(${replayGradientStepsPrediction});
</#if>
	