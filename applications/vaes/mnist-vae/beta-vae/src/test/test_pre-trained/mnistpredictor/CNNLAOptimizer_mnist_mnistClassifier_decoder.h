/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNLAOPTIMIZER_MNIST_MNISTCLASSIFIER_DECODER
#define CNNLAOPTIMIZER_MNIST_MNISTCLASSIFIER_DECODER

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_mnist_mnistClassifier_decoder{
private:
    Optimizer *optimizerHandle;
    std::string context_name = "cpu";
            
public:
    explicit CNNLAOptimizer_mnist_mnistClassifier_decoder(){
    
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
#endif // CNNLAOPTIMIZER_MNIST_MNISTCLASSIFIER_DECODER
    
