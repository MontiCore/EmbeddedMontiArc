<#-- (c) https://github.com/MontiCore/monticore -->
<#if (config.replayMemory)??>
            'method': '${config.replayMemoryName}',
<#if (config.replayMemoryParameters['memory_size'])??>
            'memory_size': ${config.replayMemoryParameters['memory_size']},
</#if>
<#if (config.replayMemoryParameters['sample_size'])??>
            'sample_size': ${config.replayMemoryParameters['sample_size']},
</#if>
<#else>
            'method': 'online',
</#if>
            'state_dtype': 'float32',
            'action_dtype': <#if config.rlAlgorithm=="dqn">'uint8'<#else>'float32'</#if>,
            'rewards_dtype': 'float32'

