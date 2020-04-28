#ifndef CNNCALCULATOR_CONNECTOR
#define CNNCALCULATOR_CONNECTOR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "cNNCalculator_connector_predictor1.h"
#include "cNNCalculator_connector_predictor2.h"
#include "cNNCalculator_connector_predictor3.h"
#include "cNNCalculator_connector_predictor4.h"
#include "cNNCalculator_connector_predictor5.h"
#include "cNNCalculator_connector_predictor6.h"
#include "cNNCalculator_connector_cal.h"
using namespace arma;
class cNNCalculator_connector{
public:
icube image1;
icube image2;
icube image3;
icube image4;
icube image5;
icube image6;
int res;
cNNCalculator_connector_predictor1 predictor1;
cNNCalculator_connector_predictor2 predictor2;
cNNCalculator_connector_predictor3 predictor3;
cNNCalculator_connector_predictor4 predictor4;
cNNCalculator_connector_predictor5 predictor5;
cNNCalculator_connector_predictor6 predictor6;
cNNCalculator_connector_cal cal;
void init()
{
image1 = icube(1, 28, 28);
image2 = icube(1, 28, 28);
image3 = icube(1, 28, 28);
image4 = icube(1, 28, 28);
image5 = icube(1, 28, 28);
image6 = icube(1, 28, 28);
predictor1.init();
predictor2.init();
predictor3.init();
predictor4.init();
predictor5.init();
predictor6.init();
cal.init();
}
void execute()
{
predictor1.data = image1;
predictor1.execute();
predictor2.data = image2;
predictor2.execute();
predictor3.data = image3;
predictor3.execute();
predictor4.data = image4;
predictor4.execute();
predictor5.data = image5;
predictor5.execute();
predictor6.data = image6;
predictor6.execute();
cal.in1_1 = predictor1.softmax;
cal.in1_2 = predictor2.softmax;
cal.in1_3 = predictor3.softmax;
cal.in2_1 = predictor4.softmax;
cal.in2_2 = predictor5.softmax;
cal.in2_3 = predictor6.softmax;
cal.execute();
res = cal.out1;
}

};
#endif
