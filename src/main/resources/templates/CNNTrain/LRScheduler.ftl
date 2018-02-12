<#if config.python>
<#if config.lrPolicy == "step">
mx.lr_scheduler.FactorScheduler(step=${config.stepSize.get()}<#if config.learningRateDecay.present>, factor=${config.learningRateDecay.get()}</#if><#if config.minLearningRate.present>, stop_factor_lr=${config.minLearningRate.get()}</#if>)
<#elseif config.lrPolicy == "multistep">
mx.lr_scheduler.MultiFactorScheduler(step=${config.stepList.get()}<#if config.learningRateDecay.present>, factor=${config.learningRateDecay.get()}</#if>)
</#if>
<#elseif config.cpp>
//todo:
</#if>