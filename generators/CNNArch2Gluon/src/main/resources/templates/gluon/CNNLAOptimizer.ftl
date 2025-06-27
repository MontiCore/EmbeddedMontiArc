<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
/* (c) https://github.com/MontiCore/monticore */
<#list configurations as config>
#ifndef CNNLAOPTIMIZER_${config.instanceName?upper_case}
#define CNNLAOPTIMIZER_${config.instanceName?upper_case}

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_${config.instanceName}{
private:
    Optimizer *optimizerHandle;
<#if (config.context)??>
    std::string context_name = "${config.context}";
<#else>
    std::string context_name = "cpu";
</#if>
            
public:
    explicit CNNLAOptimizer_${config.instanceName}(){
    
<#if (config.optimizer)??>
<#if config.optimizerName == "adamw">
        optimizerHandle = OptimizerRegistry::Find("adam");    
<#else>    
        optimizerHandle = OptimizerRegistry::Find("${config.optimizerName}");
</#if> 
<#list config.optimizerParameters?keys as param>
<#if param == "learning_rate">
        optimizerHandle->SetParam("lr", ${config.optimizerParameters[param]});
<#elseif param == "weight_decay">
        optimizerHandle->SetParam("wd", ${config.optimizerParameters[param]});
<#elseif param == "learning_rate_policy">
        optimizerHandle->SetParam("learning_rate_policy", '${config.optimizerParameters[param]}');
<#elseif param == "learning_rate_decay">
<#assign learningRateDecay = config.optimizerParameters[param]>
<#elseif param == "learning_rate_minimum">
<#assign minLearningRate = config.optimizerParameters[param]>
<#elseif param == "step_size">
<#assign stepSize = config.optimizerParameters[param]>
<#elseif param == "learning_rate_range">
        double lr_arr [] = { <#list config.optimizerParameters['learning_rate_range'] as lr>${lr}<#if lr?has_next>, </#if></#list> };
        optimizerHandle->SetParam("lr_range", lr_arr);
<#elseif param == "weight_decay_range">
        double wd_arr [] = { <#list config.optimizerParameters['weight_decay_range'] as lr>${lr}<#if lr?has_next>, </#if></#list> };
        optimizerHandle->SetParam("wd_range", wd_arr);
<#elseif param == "momentum_range">
        double m_arr [] = { <#list config.optimizerParameters['momentum_range'] as lr>${lr}<#if lr?has_next>, </#if></#list>};
        optimizerHandle->SetParam("momentum_range", m_arr);
<#elseif param == "optimizer_options">
        std::string opt_arr [] = { <#list config.optimizerParameters['optimizer_options'] as lr>"${lr}"<#if lr?has_next>, </#if></#list> };
        optimizerHandle->SetParam("optimizer_options", opt_arr);
<#elseif param == "with_cleaning">
        optimizerHandle->SetParam("with_cleaning", "${config.optimizerParameters[param]}");
<#else>
        optimizerHandle->SetParam("${param}", ${config.optimizerParameters[param]});
</#if>
</#list>
<#if (learningRateDecay)?? && (stepSize)??>
<#if !(minLearningRate)??>
<#assign minLearningRate = "1e-08">       
</#if>
        std::unique_ptr<LRScheduler> lrScheduler(new FactorScheduler(${stepSize}, ${learningRateDecay}, ${minLearningRate}));
        optimizerHandle->SetLRScheduler(std::move(lrScheduler));
</#if>
<#elseif (config.actorOptimizer)??>
<#if config.actorOptimizerName == "adamw">
        optimizerHandle = OptimizerRegistry::Find("adam");
<#else>
        optimizerHandle = OptimizerRegistry::Find("${config.actorOptimizerName}");
</#if>
<#list config.actorOptimizerParameters?keys as param>
<#if param == "learning_rate">
        optimizerHandle->SetParam("lr", ${config.actorOptimizerParameters[param]});
<#elseif param == "weight_decay">
        optimizerHandle->SetParam("wd", ${config.actorOptimizerParameters[param]});
<#elseif param == "learning_rate_policy">
    optimizerHandle->SetParam("learning_rate_policy", '${config.actorOptimizerParameters[param]}');
<#elseif param == "learning_rate_decay">
<#assign learningRateDecay = config.actorOptimizerParameters[param]>
<#elseif param == "learning_rate_minimum">
<#assign minLearningRate = config.actorOptimizerParameters[param]>
<#elseif param == "step_size">
<#assign stepSize = config.actorOptimizerParameters[param]>
<#else>
        optimizerHandle->SetParam("${param}", ${config.actorOptimizerParameters[param]});
</#if>
</#list>
<#if (learningRateDecay)?? && (stepSize)??>
<#if !(minLearningRate)??>
<#assign minLearningRate = "1e-08">
</#if>
        std::unique_ptr<LRScheduler> lrScheduler(new FactorScheduler(${stepSize}, ${learningRateDecay}, ${minLearningRate}));
        optimizerHandle->SetLRScheduler(std::move(lrScheduler));
</#if>
<#elseif (config.criticOptimizer)??>
<#if config.criticOptimizerName == "adamw">
        optimizerHandle = OptimizerRegistry::Find("adam");
<#else>
        optimizerHandle = OptimizerRegistry::Find("${config.criticOptimizerName}");
</#if>
<#list config.criticOptimizerParameters?keys as param>
<#if param == "learning_rate">
        optimizerHandle->SetParam("lr", ${config.criticOptimizerParameters[param]});
<#elseif param == "weight_decay">
        optimizerHandle->SetParam("wd", ${config.criticOptimizerParameters[param]});
<#elseif param == "learning_rate_policy">
    optimizerHandle->SetParam("learning_rate_policy", '${config.criticOptimizerParameters[param]}');
<#elseif param == "learning_rate_decay">
<#assign learningRateDecay = config.criticOptimizerParameters[param]>
<#elseif param == "learning_rate_minimum">
<#assign minLearningRate = config.criticOptimizerParameters[param]>
<#elseif param == "step_size">
<#assign stepSize = config.criticOptimizerParameters[param]>
<#else>
        optimizerHandle->SetParam("${param}", ${config.criticOptimizerParameters[param]});
</#if>
</#list>
<#if (learningRateDecay)?? && (stepSize)??>
<#if !(minLearningRate)??>
<#assign minLearningRate = "1e-08">
</#if>
        std::unique_ptr<LRScheduler> lrScheduler(new FactorScheduler(${stepSize}, ${learningRateDecay}, ${minLearningRate}));
        optimizerHandle->SetLRScheduler(std::move(lrScheduler));
</#if>



<#else>
       optimizerHandle = OptimizerRegistry::Find("adam"); 
       optimizerHandle->SetParam("lr", 0.001);
</#if>
           
    
    
<#if (config.clipGlobalGradNorm)??>
        //clip_global_grad_norm=${config.clipGlobalGradNorm},
</#if>
    
    }
    
    
   Optimizer *getOptimizer(){
        return optimizerHandle;
   }
         
   std::string getContextName(){
        return context_name;
   }
};
#endif // CNNLAOPTIMIZER_${config.instanceName?upper_case}
    
</#list>