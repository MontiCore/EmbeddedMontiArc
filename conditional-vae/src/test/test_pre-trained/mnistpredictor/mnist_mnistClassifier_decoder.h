#ifndef MNIST_MNISTCLASSIFIER_DECODER
#define MNIST_MNISTCLASSIFIER_DECODER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CNNPredictor_mnist_mnistClassifier_decoder.h"
#include "CNNTranslator.h"
using namespace arma;
class mnist_mnistClassifier_decoder{
public:
CNNPredictor_mnist_mnistClassifier_decoder_0 _predictor_0_;
colvec encoding;
ivec label;
cube data;
void init()
{
encoding=colvec(2);
label=ivec(1);
data = cube(1, 28, 28);
}
void execute(){
    vector<float> encoding_ = CNNTranslator::translate(encoding);
    vector<float> label_ = CNNTranslator::translate(label);

    vector<float> data_(1 * 28 * 28);


    _predictor_0_.predict(encoding_, label_, data_);

    data = CNNTranslator::translateToCube(data_, std::vector<size_t> {1, 28, 28});

}

};
#endif
