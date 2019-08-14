#ifndef CHEETAH_MASTER_CHEETAH
#define CHEETAH_MASTER_CHEETAH
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CNNPredictor_cheetah_master_cheetah.h"
#include "CNNTranslator.h"
using namespace arma;
class cheetah_master_cheetah{
public:
CNNPredictor_cheetah_master_cheetah_0 _predictor_0_;
colvec state;
colvec action;
void init()
{
state=colvec(26);
action=colvec(6);
}
void execute(){
    vector<float> CNN_action(6);

    _predictor_0_.predict(CNNTranslator::translate(state),
                CNN_action);

    action = CNNTranslator::translateToCol(CNN_action, std::vector<size_t> {6});

}

};
#endif
