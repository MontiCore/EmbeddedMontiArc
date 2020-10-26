#ifndef CNNLAOPTIMIZER_MEMORIESWITHPRODUCTKEYS_CONNECTOR_PREDICTOR
#define CNNLAOPTIMIZER_MEMORIESWITHPRODUCTKEYS_CONNECTOR_PREDICTOR

#include <mxnet-cpp/MxNetCpp.h>

#include <string>
#include <vector>
#include <memory>

using namespace mxnet::cpp;    

class CNNLAOptimizer_memorieswithproductkeys_connector_predictor{
private:
    Optimizer *optimizerHandle;
    std::string context_name = "cpu";
            
public:
    explicit CNNLAOptimizer_memorieswithproductkeys_connector_predictor(){
    
        optimizerHandle = OptimizerRegistry::Find("adam");
        optimizerHandle->SetParam("wd", 0.01);
        optimizerHandle->SetParam("lr", 3.0E-5);
           
    
    
    
    }
    
    
   Optimizer *getOptimizer(){
        return optimizerHandle;
   }
         
   std::string getContextName(){
        return context_name;
   }
};
#endif // CNNLAOPTIMIZER_MEMORIESWITHPRODUCTKEYS_CONNECTOR_PREDICTOR
    
