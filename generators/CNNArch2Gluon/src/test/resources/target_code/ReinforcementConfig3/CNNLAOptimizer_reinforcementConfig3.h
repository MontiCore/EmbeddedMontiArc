/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNLAOPTIMIZER_REINFORCEMENTCONFIG3
#define CNNLAOPTIMIZER_REINFORCEMENTCONFIG3

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_reinforcementConfig3{
private:
    Optimizer *optimizerHandle;
    std::string context_name = "cpu";
            
public:
    explicit CNNLAOptimizer_reinforcementConfig3(){
    
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
#endif // CNNLAOPTIMIZER_REINFORCEMENTCONFIG3
    
