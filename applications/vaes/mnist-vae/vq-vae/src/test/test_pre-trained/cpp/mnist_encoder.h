#ifndef MNIST_ENCODER
#define MNIST_ENCODER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CNNPredictor_mnist_encoder.h"
#include "CNNTranslator.h"
using namespace arma;
class mnist_encoder{
public:
CNNPredictor_mnist_encoder_0 _predictor_0_;
cube encoding;
cube data;
void init()
{
data = cube(1, 28, 28);
encoding = cube(16, 7, 7);
}
void execute(){
    vector<float> data_ = CNNTranslator::translate(data);

    vector<float> encoding_(16 * 7 * 7);


    _predictor_0_.predict(data_, encoding_);

    encoding = CNNTranslator::translateToCube(encoding_, std::vector<size_t> {16, 7, 7});

}

};
#endif
