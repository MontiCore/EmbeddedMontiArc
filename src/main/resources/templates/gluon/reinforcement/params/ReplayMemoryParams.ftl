<#if (config.replayMemory)??>
            'method': '${config.replayMemory.method}',
<#if (config.replayMemory.memory_size)??>
            'memory_size': ${config.replayMemory.memory_size},
</#if>
<#if (config.replayMemory.sample_size)??>
            'sample_size': ${config.replayMemory.sample_size},
</#if>
<#else>
            'method': 'online',
</#if>
            'state_dtype': 'float32',
            'action_dtype': <#if config.rlAlgorithm=="dqn">'uint8'<#else>'float32'</#if>,
            'rewards_dtype': 'float32'
