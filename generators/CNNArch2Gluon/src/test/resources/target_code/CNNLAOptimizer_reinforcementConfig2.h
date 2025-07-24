/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNLAOPTIMIZER_REINFORCEMENTCONFIG2
#define CNNLAOPTIMIZER_REINFORCEMENTCONFIG2

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_reinforcementConfig2{
private:
    Optimizer *optimizerHandle;
    std::string context_name = "cpu";
            
public:
    explicit CNNLAOptimizer_reinforcementConfig2(){
    
        optimizerHandle = OptimizerRegistry::Find("rmsprop");
        optimizerHandle->SetParam("wd", 0.01);
        optimizerHandle->SetParam("centered", True);
        optimizerHandle->SetParam("gamma2", 0.9);
        optimizerHandle->SetParam("gamma1", 0.9);
        optimizerHandle->SetParam("clip_weights", 10.0);
        optimizerHandle->SetParam("epsilon", 1.0E-6);
        optimizerHandle->SetParam("rescale_grad", 1.1);
        optimizerHandle->SetParam("clip_gradient", 10.0);
    optimizerHandle->SetParam("learning_rate_policy", 'step');
        optimizerHandle->SetParam("lr", 0.001);
        std::unique_ptr<LRScheduler> lrScheduler(new FactorScheduler(1000, 0.9, 1.0E-5));
        optimizerHandle->SetLRScheduler(std::move(lrScheduler));
           
    
    
    
    }
    
    
   Optimizer *getOptimizer(){
        return optimizerHandle;
   }
         
   std::string getContextName(){
        return context_name;
   }
};
#endif // CNNLAOPTIMIZER_REINFORCEMENTCONFIG2
    
