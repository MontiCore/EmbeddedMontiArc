/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNLAOPTIMIZER_MNIST_MNISTCLASSIFIER_NET
#define CNNLAOPTIMIZER_MNIST_MNISTCLASSIFIER_NET

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_mnist_mnistClassifier_net{
private:
    Optimizer *optimizerHandle;
    std::string context_name = "cpu";
            
public:
    explicit CNNLAOptimizer_mnist_mnistClassifier_net(){
    
        optimizerHandle = OptimizerRegistry::Find("adam");
        optimizerHandle->SetParam("epsilon", 1.0E-8);
        optimizerHandle->SetParam("wd", 0.001);
        optimizerHandle->SetParam("beta1", 0.9);
        optimizerHandle->SetParam("beta2", 0.999);
        optimizerHandle->SetParam("lr", 0.001);
           
    
    
    
    }
    
    
   Optimizer *getOptimizer(){
        return optimizerHandle;
   }
         
   std::string getContextName(){
        return context_name;
   }
};
#endif // CNNLAOPTIMIZER_MNIST_MNISTCLASSIFIER_NET
    
