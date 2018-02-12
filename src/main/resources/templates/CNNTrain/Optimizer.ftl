<#if config.python>
mx.optimizer.create('${config.optimizer}',
              begin_num_update=initial_step<#if config.learningRate.present>,
              learning_rate=${config.learningRate.get()}</#if><#if config.lrPolicy != "fixed">,
              lr_scheduler=${tc.include("LRScheduler")}</#if><#if config.weightDecay.present>,
              wd=${config.weightDecay.get()}</#if>,
              rescale_grad=<#if config.rescaleGrad.present>${config.rescaleGrad.get()}<#else>1.0/${config.batchsize}</#if><#if config.clipGradient.present>,
              clip_gradient=${config.clipGradient.get()}</#if><#if config.momentum.present>,
              momentum=${config.momentum.get()}</#if><#if config.beta1.present>,
              beta1=${config.beta1.get()}</#if><#if config.beta2.present>,
              beta2=${config.beta2.get()}</#if><#if config.epsilon.present>,
              epsilon=${config.epsilon.get()}</#if><#if config.eps.present>,
              eps=${config.eps.get()}</#if><#if config.gamma1.present>,
              gamma1=${config.gamma1.get()}</#if><#if config.gamma2.present>,
              gamma2=${config.gamma2.get()}</#if><#if config.centered.present>,
              centered=${config.centered.get()}</#if><#if config.clipWeights.present>,
              clip_weights=${config.clipWeights.get()}</#if><#if config.rho.present>,
              rho=${config.rho.get()}</#if>)
<#elseif config.cpp>
//todo:
</#if>