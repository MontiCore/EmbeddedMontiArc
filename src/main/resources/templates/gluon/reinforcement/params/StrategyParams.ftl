<#-- (c) https://github.com/MontiCore/monticore -->
<#if (config.strategy)??>
            'method':'${config.strategyName}',
</#if>
<#if (config.strategyParameters['epsilon'])??>
            'epsilon': ${config.strategyParameters['epsilon']},
</#if>
<#if (config.strategyParameters['min_epsilon'])??>
            'min_epsilon': ${config.strategyParameters['min_epsilon']},
</#if>
<#if (config.strategyParameters['epsilon_decay_method'])??>
            'epsilon_decay_method': '${config.strategyParameters['epsilon_decay_method']}',
</#if>
<#if (config.strategyParameters['epsilon_decay'])??>
            'epsilon_decay': ${config.strategyParameters['epsilon_decay']},
</#if>
<#if (config.strategyParameters['epsilon_decay_start'])??>
            'epsilon_decay_start': ${config.strategyParameters['epsilon_decay_start']},
</#if>
<#if (config.strategyParameters['epsilon_decay_per_step'])??>
            'epsilon_decay_per_step': ${config.strategyParameters['epsilon_decay_per_step']},
</#if>
<#if (config.strategyName=="gaussian")>
    <#if (config.strategyParameters['noise_variance'])??>
            'noise_variance': ${config.strategyParameters['noise_variance']},
    </#if>
</#if>
<#if (config.strategyName)?? && (config.strategyName=="ornstein_uhlenbeck" || config.strategyName=="gaussian")>
    <#if (config.strategyParameters['action_low'])?? >
            'action_low': ${config.strategyParameters['action_low']},
    <#else>
            'action_low': -np.infty,
    </#if>
    <#if (config.strategyParameters['action_high'])?? >
            'action_high': ${config.strategyParameters['action_high']},
    <#else>
            'action_high' : np.infty,
    </#if>
</#if>
<#if (config.strategyName)?? && (config.strategyName=="ornstein_uhlenbeck")>
    <#if (config.strategyParameters['mu'])??>
            'mu': [<#list config.strategyParameters['mu'] as m>${m}<#if m?has_next>, </#if></#list>],
    </#if>
    <#if (config.strategyParameters['theta'])??>
            'theta': [<#list config.strategyParameters['theta'] as t>${t}<#if t?has_next>, </#if></#list>],
    </#if>
    <#if (config.strategyParameters['sigma'])??>
            'sigma': [<#list config.strategyParameters['sigma'] as s>${s}<#if s?has_next>, </#if></#list>],
    </#if>
</#if>