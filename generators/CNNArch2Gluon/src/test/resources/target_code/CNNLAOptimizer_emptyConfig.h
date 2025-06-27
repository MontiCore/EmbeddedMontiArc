/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNLAOPTIMIZER_EMPTYCONFIG
#define CNNLAOPTIMIZER_EMPTYCONFIG

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_emptyConfig{
private:
    Optimizer *optimizerHandle;
    std::string context_name = "cpu";
            
public:
    explicit CNNLAOptimizer_emptyConfig(){
    
       optimizerHandle = OptimizerRegistry::Find("adam"); 
       optimizerHandle->SetParam("lr", 0.001);
           
    
    
    
    }
    
    
   Optimizer *getOptimizer(){
        return optimizerHandle;
   }
         
   std::string getContextName(){
        return context_name;
   }
};
#endif // CNNLAOPTIMIZER_EMPTYCONFIG
    
