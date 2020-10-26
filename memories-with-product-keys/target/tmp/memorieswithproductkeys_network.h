#ifndef MEMORIESWITHPRODUCTKEYS_NETWORK
#define MEMORIESWITHPRODUCTKEYS_NETWORK
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CNNPredictor_memorieswithproductkeys_network.h"
#include "CNNTranslator.h"
using namespace arma;
class memorieswithproductkeys_network{
public:
CNNPredictor_memorieswithproductkeys_network_0 _predictor_0_;
ivec data_0;
ivec data_1;
ivec data_2;
colvec softmax;
void init()
{
data_0=ivec(5);
data_1=ivec(5);
data_2=ivec(1);
softmax=colvec(33);
}
void execute(){
    vector<float> data_0_ = CNNTranslator::translate(data_0);
    vector<float> data_1_ = CNNTranslator::translate(data_1);
    vector<float> data_2_ = CNNTranslator::translate(data_2);

    vector<float> softmax_(33);


    _predictor_0_.predict(data_0_, data_1_, data_2_, softmax_);

    softmax = CNNTranslator::translateToCol(softmax_, std::vector<size_t> {33});

}

};
#endif
