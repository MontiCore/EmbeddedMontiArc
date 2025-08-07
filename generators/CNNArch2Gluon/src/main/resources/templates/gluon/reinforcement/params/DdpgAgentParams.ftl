<#-- (c) https://github.com/MontiCore/monticore -->
        'actor': actor_creator.networks[0],
        'critic': critic_creator.networks[0],
<#if (config.softTargetUpdateRate)??>
        'soft_target_update_rate': ${config.softTargetUpdateRate},
</#if>
<#if (config.actorOptimizer)??>
        'actor_optimizer': '${config.actorOptimizerName}',
        'actor_optimizer_params': {
<#list config.actorOptimizerParameters?keys as param>
        <#assign paramName = param>
        <#assign paramValue = config.actorOptimizerParameters[param]>
        <#if param == "learning_rate_policy">
            <#assign paramValue = "'${paramValue}'">
        </#if>
        '${paramName}': ${paramValue}<#sep>,
</#list>},
</#if>
<#if (config.criticOptimizer)??>
        'critic_optimizer': '${config.criticOptimizerName}',
        'critic_optimizer_params': {
<#list config.criticOptimizerParameters?keys as param>
        <#assign paramName = param>
        <#assign paramValue = config.criticOptimizerParameters[param]>
        <#if param == "learning_rate_policy">
            <#assign paramValue = "'${paramValue}'">
        </#if>
        '${paramName}': ${paramValue}<#sep>,
</#list>},
</#if>