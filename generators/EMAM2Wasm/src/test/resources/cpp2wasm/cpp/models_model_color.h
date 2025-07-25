/* (c) https://github.com/MontiCore/monticore */
#ifndef MODELS_MODEL_COLOR
#define MODELS_MODEL_COLOR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class models_model_color{
const int width = 640;
const int height = 480;
public:
mat red;
mat green;
mat blue;
mat alpha;
void init()
{
red=mat(width,height);
green=mat(width,height);
blue=mat(width,height);
alpha=mat(width,height);
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
