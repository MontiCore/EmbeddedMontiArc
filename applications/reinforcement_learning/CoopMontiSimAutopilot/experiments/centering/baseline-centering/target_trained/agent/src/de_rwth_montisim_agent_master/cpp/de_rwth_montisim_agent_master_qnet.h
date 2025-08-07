#ifndef DE_RWTH_MONTISIM_AGENT_MASTER_QNET
#define DE_RWTH_MONTISIM_AGENT_MASTER_QNET
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CNNPredictor_de_rwth_montisim_agent_master_qnet.h"
#include "CNNTranslator.h"
using namespace arma;
class de_rwth_montisim_agent_master_qnet{
public:
CNNPredictor_de_rwth_montisim_agent_master_qnet_0 _predictor_0_;
colvec state;
colvec action;
void init()
{
state=colvec(50);
action=colvec(3);
}
void execute(){
    vector<float> state_ = CNNTranslator::translate(state);

    vector<float> action_(3);


    _predictor_0_.predict(state_, action_);

    action = CNNTranslator::translateToCol(action_, std::vector<size_t> {3});

}

};
#endif
