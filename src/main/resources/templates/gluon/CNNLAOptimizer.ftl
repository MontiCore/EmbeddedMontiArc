<#-- (c) https://github.com/MontiCore/monticore -->
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
    
<#if (config.configuration.optimizer)??>
<#if config.optimizerName == "adamw">
        optimizerHandle = OptimizerRegistry::Find("adam");    
<#else>    
        optimizerHandle = OptimizerRegistry::Find("${config.optimizerName}");
</#if> 
<#list config.optimizerParams?keys as param>
<#if param == "learning_rate">
        optimizerHandle->SetParam("lr", ${config.optimizerParams[param]});
<#elseif param == "weight_decay">
        optimizerHandle->SetParam("wd", ${config.optimizerParams[param]});
<#elseif param == "learning_rate_decay">
<#assign learningRateDecay = config.optimizerParams[param]>
<#elseif param == "learning_rate_minimum">
<#assign minLearningRate = config.optimizerParams[param]> 
<#elseif param == "step_size">
<#assign stepSize = config.optimizerParams[param]>     
<#else>
        optimizerHandle->SetParam("${param}", ${config.optimizerParams[param]});
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
