<#-- (c) https://github.com/MontiCore/monticore -->
        'qnet':qnet_creator.networks[0],
<#if (config.useFixTargetNetwork)?? && config.useFixTargetNetwork>
        'use_fix_target': True,
        'target_update_interval': ${config.targetNetworkUpdateInterval},
<#else>
        'use_fix_target': False,
</#if>
<#if (config.loss)??>
        'loss_function': '${config.lossName}',
<#if (config.lossParameters)??>
        'loss_params': {
<#list config.lossParameters?keys as param>
            '${param}': ${config.lossParameters[param]}<#sep>,
</#list>
},
</#if>
</#if>
<#if (config.optimizer)?? && (config.optimizerName) != "hpo">
        'optimizer': '${config.optimizerName}',
        'optimizer_params': {
<#list config.optimizerParameters?keys as param>
                <#assign paramName = param>
                <#assign paramValue = config.optimizerParameters[param]>
                <#if param == "learning_rate_policy">
                        <#assign paramValue = "'${paramValue}'">
                </#if>
                '${paramName}': ${paramValue}<#sep>,
</#list>
        },
</#if>
<#if (config.useDoubleDqn)?? && config.useDoubleDqn>
        'double_dqn': True,
<#else>
        'double_dqn': False,
</#if>