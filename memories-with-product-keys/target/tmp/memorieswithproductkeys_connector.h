#ifndef MEMORIESWITHPRODUCTKEYS_CONNECTOR
#define MEMORIESWITHPRODUCTKEYS_CONNECTOR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "memorieswithproductkeys_connector_predictor.h"
#include "memorieswithproductkeys_connector_argMax.h"
using namespace arma;
class memorieswithproductkeys_connector{
public:
ivec data_0;
ivec data_1;
ivec data_2;
int res;
memorieswithproductkeys_connector_predictor predictor;
memorieswithproductkeys_connector_argMax argMax;
void init()
{
data_0=ivec(5);
data_1=ivec(5);
data_2=ivec(1);
predictor.init();
argMax.init();
}
void execute()
{
predictor.data_0 = data_0;
predictor.data_1 = data_1;
predictor.data_2 = data_2;
predictor.execute();
argMax.inputVector = predictor.softmax;
argMax.execute();
res = argMax.maxIndex;
}

};
#endif
