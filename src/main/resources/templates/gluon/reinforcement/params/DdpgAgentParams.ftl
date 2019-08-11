        'actor': actor_creator.networks[0],
        'critic': critic_creator.networks[0],
<#if (config.softTargetUpdateRate)??>
        'soft_target_update_rate': ${config.softTargetUpdateRate},
</#if>
<#if (config.configuration.optimizer)??>
        'actor_optimizer': '${config.optimizerName}',
        'actor_optimizer_params': {
<#list config.optimizerParams?keys as param>
            '${param}': ${config.optimizerParams[param]}<#sep>,
</#list>},
</#if>
<#if (config.configuration.criticOptimizer)??>
        'critic_optimizer': '${config.criticOptimizerName}',
        'critic_optimizer_params': {
<#list config.criticOptimizerParams?keys as param>
            '${param}': ${config.criticOptimizerParams[param]}<#sep>,
</#list>},
</#if>
