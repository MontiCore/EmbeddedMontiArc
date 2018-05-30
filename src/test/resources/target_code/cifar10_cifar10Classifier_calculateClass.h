#ifndef CIFAR10_CIFAR10CLASSIFIER_CALCULATECLASS
#define CIFAR10_CIFAR10CLASSIFIER_CALCULATECLASS
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class cifar10_cifar10Classifier_calculateClass{
const int n = 10;
public:
colvec inputVector;
double maxIndex;
void init()
{
inputVector=colvec(n);
}
void execute()
{
maxIndex = 0;
double maxValue = inputVector(0);
for( auto i=1;i<=(n-1);++i){
if((inputVector(i) > maxValue)){
maxIndex = i;
maxValue = inputVector(i);
}
}
}

};
#endif
