#ifndef TORCS_AGENT_TORCSAGENT_DQN
#define TORCS_AGENT_TORCSAGENT_DQN
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CNNPredictor_torcs_agent_torcsAgent_dqn.h"
#include "CNNTranslator.h"
using namespace arma;
class torcs_agent_torcsAgent_dqn{
const int discrete_actions = 30;
public:
CNNPredictor_torcs_agent_torcsAgent_dqn _cnn_;
colvec state;
colvec qvalues;
void init()
{
state=colvec(5);
qvalues=colvec(discrete_actions);
}
void execute(){
    vector<float> CNN_qvalues(30);

    _cnn_.predict(CNNTranslator::translate(state),
                CNN_qvalues);

    qvalues = CNNTranslator::translateToCol(CNN_qvalues, std::vector<size_t> {30});

}

};
#endif
