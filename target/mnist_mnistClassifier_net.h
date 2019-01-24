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
CNNPredictor_mnist_mnistClassifier_net _cnn_;
icube image;
colvec predictions;
void init()
{
image = icube(1, 28, 28);
predictions=colvec(classes);
}
void execute(){
    vector<float> CNN_predictions(10);

    _cnn_.predict(CNNTranslator::translate(image),
                CNN_predictions);

    predictions = CNNTranslator::translateToCol(CNN_predictions, std::vector<size_t> {10});

}

};
#endif
