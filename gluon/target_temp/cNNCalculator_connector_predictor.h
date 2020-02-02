#ifndef CNNCALCULATOR_CONNECTOR_PREDICTOR
#define CNNCALCULATOR_CONNECTOR_PREDICTOR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CNNPredictor_cNNCalculator_connector_predictor.h"
#include "CNNTranslator.h"
using namespace arma;
class cNNCalculator_connector_predictor{
public:
CNNPredictor_cNNCalculator_connector_predictor_0 _predictor_0_;
cube noise;
cube image;
void init()
{
noise = cube(100, 1, 1);
image = cube(1, 64, 64);
}
void execute(){
    vector<float> noise_ = CNNTranslator::translate(noise);

    vector<float> image_(1 * 64 * 64);


    _predictor_0_.predict(noise_, image_);

    image = CNNTranslator::translateToCube(image_, std::vector<size_t> {1, 64, 64});

}

};
#endif
