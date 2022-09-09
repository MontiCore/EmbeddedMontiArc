/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNLAOPTIMIZER_DE_RWTH_MONTISIM_AGENT_MASTER_QNET
#define CNNLAOPTIMIZER_DE_RWTH_MONTISIM_AGENT_MASTER_QNET

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_de_rwth_montisim_agent_master_qnet{
private:
    Optimizer *optimizerHandle;
    std::string context_name = "gpu";
            
public:
    explicit CNNLAOptimizer_de_rwth_montisim_agent_master_qnet(){
    
        optimizerHandle = OptimizerRegistry::Find("adam");
        optimizerHandle->SetParam("lr", 1.0E-4);
           
    
    
    
    }
    
    
   Optimizer *getOptimizer(){
        return optimizerHandle;
   }
         
   std::string getContextName(){
        return context_name;
   }
};
#endif // CNNLAOPTIMIZER_DE_RWTH_MONTISIM_AGENT_MASTER_QNET
    
