/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNLAOPTIMIZER_DEFAULTGAN
#define CNNLAOPTIMIZER_DEFAULTGAN

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_defaultGAN{
private:
    Optimizer *optimizerHandle;
    std::string context_name = "cpu";
            
public:
    explicit CNNLAOptimizer_defaultGAN(){
    
        optimizerHandle = OptimizerRegistry::Find("adam");
        optimizerHandle->SetParam("beta1", 0.5);
        optimizerHandle->SetParam("lr", 2.0E-4);
           
    
    
    
    }
    
    
   Optimizer *getOptimizer(){
        return optimizerHandle;
   }
         
   std::string getContextName(){
        return context_name;
   }
};
#endif // CNNLAOPTIMIZER_DEFAULTGAN
    
