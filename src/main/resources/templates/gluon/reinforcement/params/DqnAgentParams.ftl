        'qnet':qnet_creator.net,
<#if (config.useFixTargetNetwork)?? && config.useFixTargetNetwork>
        'use_fix_target': True,
        'target_update_interval': ${config.targetNetworkUpdateInterval},
<#else>
        'use_fix_target': False,
</#if>
<#if (config.loss)??>
        'loss_function': '${config.loss}',
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
