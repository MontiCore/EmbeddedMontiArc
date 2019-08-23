/* (c) https://github.com/MontiCore/monticore */
#ifndef MODELS_MODEL
#define MODELS_MODEL
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "models_model_color.h"
using namespace arma;
class models_model{
public:
mat rgba[4];
models_model_color color;
void init()
{
rgba[0]=mat(640,480);
rgba[1]=mat(640,480);
rgba[2]=mat(640,480);
rgba[3]=mat(640,480);
color.init();
}
void execute()
{
color.execute();
rgba[0] = color.red;
rgba[1] = color.green;
rgba[1] = color.blue;
rgba[3] = color.alpha;
}

};
#endif
