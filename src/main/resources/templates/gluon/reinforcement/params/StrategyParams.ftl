<#if (config.strategy)??>
            'method':'${config.strategy.method}',
</#if>
<#if (config.strategy.epsilon)??>
            'epsilon': ${config.strategy.epsilon},
</#if>
<#if (config.strategy.min_epsilon)??>
            'min_epsilon': ${config.strategy.min_epsilon},
</#if>
<#if (config.strategy.epsilon_decay_method)??>
            'epsilon_decay_method': '${config.strategy.epsilon_decay_method}',
</#if>
<#if (config.strategy.epsilon_decay)??>
            'epsilon_decay': ${config.strategy.epsilon_decay},
</#if>
<#if (config.strategy.epsilon_decay_start)??>
            'epsilon_decay_start': ${config.strategy.epsilon_decay_start},
</#if>
<#if (config.strategy.epsilon_decay_start)??>
            'epsilon_decay_per_step': ${config.strategy.epsilon_decay_per_step?string('True', 'False')},
</#if>
<#if (config.strategy.method)?? && (config.strategy.method=="ornstein_uhlenbeck")>
<#if (config.strategy.action_low)?? >
            'action_low': ${config.strategy.action_low},
<#else>
            'action_low': -np.infty,
</#if>
<#if (config.strategy.action_high)?? >
            'action_high': ${config.strategy.action_high},
<#else>
            'action_high' : np.infty,
</#if>
<#if (config.strategy.mu)??>
            'mu': [<#list config.strategy.mu as m>${m}<#if m?has_next>, </#if></#list>],
</#if>
<#if (config.strategy.theta)??>
            'theta': [<#list config.strategy.theta as t>${t}<#if t?has_next>, </#if></#list>],
</#if>
<#if (config.strategy.sigma)??>
            'sigma': [<#list config.strategy.sigma as s>${s}<#if s?has_next>, </#if></#list>],
</#if>
</#if>
