/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNLAOPTIMIZER_TD3CONFIG
#define CNNLAOPTIMIZER_TD3CONFIG

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_tD3Config{
private:
    Optimizer *optimizerHandle;
    std::string context_name = "cpu";
            
public:
    explicit CNNLAOptimizer_tD3Config(){
    
        optimizerHandle = OptimizerRegistry::Find("adam");
    optimizerHandle->SetParam("learning_rate_policy", 'step');
        optimizerHandle->SetParam("lr", 1.0E-4);
           
    
    
    
    }
    
    
   Optimizer *getOptimizer(){
        return optimizerHandle;
   }
         
   std::string getContextName(){
        return context_name;
   }
};
#endif // CNNLAOPTIMIZER_TD3CONFIG
    
