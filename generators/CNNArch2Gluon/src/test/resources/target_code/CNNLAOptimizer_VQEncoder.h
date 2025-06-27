/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNLAOPTIMIZER_VQENCODER
#define CNNLAOPTIMIZER_VQENCODER

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_VQEncoder{
private:
    Optimizer *optimizerHandle;
    std::string context_name = "gpu";
            
public:
    explicit CNNLAOptimizer_VQEncoder(){
    
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
#endif // CNNLAOPTIMIZER_VQENCODER
    
