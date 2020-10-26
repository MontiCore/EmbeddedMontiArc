#ifndef MEMORIESWITHPRODUCTKEYS_CONNECTOR_PREDICTOR
#define MEMORIESWITHPRODUCTKEYS_CONNECTOR_PREDICTOR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CNNPredictor_memorieswithproductkeys_connector_predictor.h"
#include "CNNTranslator.h"
using namespace arma;
class memorieswithproductkeys_connector_predictor{
const int n = 5;
public:
CNNPredictor_memorieswithproductkeys_connector_predictor_0 _predictor_0_;
ivec data_0;
ivec data_1;
ivec data_2;
colvec softmax;
void init()
{
data_0=ivec(n);
data_1=ivec(n);
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
