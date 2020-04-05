<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign replayInterval = element.replayInterval?c>
<#assign replayBatchSize = element.replayBatchSize?c>
<#assign replaySteps = element.replaySteps?c>
<#assign replayGradientSteps = element.replayGradientSteps?c>
<#assign storeProb = element.storeProb?c>
<#assign maxStoredSamples = element.maxStoredSamples?c>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = ReplayMemory(replay_interval=${replayInterval}, replay_batch_size=${replayBatchSize}, replay_steps=${replaySteps}, replay_gradient_steps=${replayGradientSteps}, store_prob =${storeProb}, max_stored_samples=${maxStoredSamples})
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(x)
</#if>
	