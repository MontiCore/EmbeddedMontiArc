#ifndef MNIST_MNISTCLASSIFIER_NET
#define MNIST_MNISTCLASSIFIER_NET
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CNNPredictor_mnist_mnistClassifier_net.h"
#include "CNNTranslator.h"
using namespace arma;
class mnist_mnistClassifier_net{
const int classes = 10;
public:
CNNPredictor_mnist_mnistClassifier_net_0 _predictor_0_;
cube data;
colvec softmax;
void init()
{
data = cube(1, 28, 28);
softmax=colvec(classes);
}
void execute(){
    vector<float> data_ = CNNTranslator::translate(data);

    vector<float> softmax_(10);


    _predictor_0_.predict(data_, softmax_);

    softmax = CNNTranslator::translateToCol(softmax_, std::vector<size_t> {10});

}

};
#endif
