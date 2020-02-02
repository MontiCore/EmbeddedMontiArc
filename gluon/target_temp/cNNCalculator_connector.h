#ifndef CNNCALCULATOR_CONNECTOR
#define CNNCALCULATOR_CONNECTOR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "cNNCalculator_connector_predictor.h"
using namespace arma;
class cNNCalculator_connector{
public:
cube noise;
cube res;
cNNCalculator_connector_predictor predictor;
void init()
{
noise = cube(100, 1, 1);
res = cube(3, 64, 64);
predictor.init();
}
void execute()
{
predictor.noise = noise;
predictor.execute();
res = predictor.image;
}

};
#endif
