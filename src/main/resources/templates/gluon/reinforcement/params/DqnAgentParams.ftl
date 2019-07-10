        'qnet':qnet_creator.networks[0],
<#if (config.useFixTargetNetwork)?? && config.useFixTargetNetwork>
        'use_fix_target': True,
        'target_update_interval': ${config.targetNetworkUpdateInterval},
<#else>
        'use_fix_target': False,
</#if>
<#if (config.configuration.loss)??>
        'loss': '${config.lossName}',
<#if (config.lossParams)??>
        'loss_params': {
<#list config.lossParams?keys as param>
            '${param}': ${config.lossParams[param]}<#sep>,
</#list>
},
</#if>
</#if>
<#if (config.configuration.optimizer)??>
        'optimizer': '${config.optimizerName}',
        'optimizer_params': {
<#list config.optimizerParams?keys as param>
            '${param}': ${config.optimizerParams[param]}<#sep>,
</#list>
        },
</#if>
<#if (config.useDoubleDqn)?? && config.useDoubleDqn>
        'double_dqn': True,
<#else>
        'double_dqn': False,
</#if>
