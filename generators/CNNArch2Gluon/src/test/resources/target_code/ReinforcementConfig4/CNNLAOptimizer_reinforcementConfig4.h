/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNLAOPTIMIZER_REINFORCEMENTCONFIG4
#define CNNLAOPTIMIZER_REINFORCEMENTCONFIG4

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_reinforcementConfig4{
private:
    Optimizer *optimizerHandle;
    std::string context_name = "cpu";
            
public:
    explicit CNNLAOptimizer_reinforcementConfig4(){
    
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
#endif // CNNLAOPTIMIZER_REINFORCEMENTCONFIG4
    
