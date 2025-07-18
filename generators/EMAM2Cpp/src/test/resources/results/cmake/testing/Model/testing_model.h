/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_MODEL
#define TESTING_MODEL
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "testing_model_modelColor.h"
using namespace arma;
class testing_model{
public:
imat rgba[4];
testing_model_modelColor modelColor;
void init()
{
rgba[0]=imat(640,480);
rgba[1]=imat(640,480);
rgba[2]=imat(640,480);
rgba[3]=imat(640,480);
modelColor.init();
}
void execute()
{
modelColor.execute();
rgba[0] = modelColor.red;
rgba[1] = modelColor.green;
rgba[2] = modelColor.blue;
rgba[3] = modelColor.alpha;
}

};
#endif
