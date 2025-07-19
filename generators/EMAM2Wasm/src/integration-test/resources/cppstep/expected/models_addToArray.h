/* (c) https://github.com/MontiCore/monticore */
#ifndef MODELS_ADDTOARRAY
#define MODELS_ADDTOARRAY
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class models_addToArray{
public:
double array[5];
double offset;
double result[5];
void init()
{
}
void execute()
{
result = offset+array;
}

};
#endif
