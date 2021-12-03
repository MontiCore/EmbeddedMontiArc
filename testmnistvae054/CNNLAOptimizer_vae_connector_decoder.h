/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNLAOPTIMIZER_VAE_CONNECTOR_DECODER
#define CNNLAOPTIMIZER_VAE_CONNECTOR_DECODER

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_vae_connector_decoder{
private:
    Optimizer *optimizerHandle;
    std::string context_name = "cpu";
            
public:
    explicit CNNLAOptimizer_vae_connector_decoder(){
    
        optimizerHandle = OptimizerRegistry::Find("adam");
        optimizerHandle->SetParam("wd", 0.01);
        optimizerHandle->SetParam("lr", 0.003);
           
    
    
    
    }
    
    
   Optimizer *getOptimizer(){
        return optimizerHandle;
   }
         
   std::string getContextName(){
        return context_name;
   }
};
#endif // CNNLAOPTIMIZER_VAE_CONNECTOR_DECODER
    
