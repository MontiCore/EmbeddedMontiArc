#ifndef MNIST_MNISTCLASSIFIER
#define MNIST_MNISTCLASSIFIER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "mnist_mnistClassifier_net.h"
#include "mnist_mnistClassifier_calculateClass.h"
#include "mnist_mnistClassifier_decoder.h"
using namespace arma;
class mnist_mnistClassifier{
public:
cube data;
ivec label;
int classIndex;
double probability;
mnist_mnistClassifier_net net;
mnist_mnistClassifier_calculateClass calculateClass;
mnist_mnistClassifier_decoder decoder;
void init()
{
data = cube(16, 7, 7);
label=ivec(1);
net.init();
calculateClass.init();
decoder.init();
}
void execute()
{
decoder.encoding = data;
decoder.execute();
net.data = decoder.data;
net.execute();
calculateClass.inputVector = net.softmax;
calculateClass.execute();
classIndex = calculateClass.maxIndex;
probability = calculateClass.maxValue;
}

};
#endif
