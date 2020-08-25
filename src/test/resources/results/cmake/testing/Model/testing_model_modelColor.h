/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_MODEL_MODELCOLOR
#define TESTING_MODEL_MODELCOLOR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class testing_model_modelColor{
const int width = 640;
const int height = 480;
public:
imat red;
imat green;
imat blue;
imat alpha;
void init()
{
red=imat(width,height);
green=imat(width,height);
blue=imat(width,height);
alpha=imat(width,height);
}
void execute()
{
red = (zeros<mat>(640, 480));
green = (zeros<mat>(640, 480));
blue = (zeros<mat>(640, 480));
alpha = (zeros<mat>(640, 480));
}

};
#endif
