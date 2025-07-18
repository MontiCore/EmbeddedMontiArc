/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNLAOPTIMIZER_MNIST_ENCODER
#define CNNLAOPTIMIZER_MNIST_ENCODER

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_mnist_encoder{
private:
    Optimizer *optimizerHandle;
    std::string context_name = "gpu";
            
public:
    explicit CNNLAOptimizer_mnist_encoder(){
    
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
#endif // CNNLAOPTIMIZER_MNIST_ENCODER
    
