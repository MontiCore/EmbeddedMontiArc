#ifndef ENDTOEND_NVIDIA
#define ENDTOEND_NVIDIA
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CNNPredictor_endtoend_nvidia.h"
#include "CNNTranslator.h"
using namespace arma;
class endtoend_nvidia{
public:
CNNPredictor_endtoend_nvidia_0 _predictor_0_;
cube data;
colvec ergebnis;
void init()
{
data = cube(9, 480, 640);
ergebnis=colvec(1);
}
void execute(){
    vector<float> CNN_ergebnis(1);

    _predictor_0_.predict(CNNTranslator::translate(data),
                CNN_ergebnis);

    ergebnis = CNNTranslator::translateToCol(CNN_ergebnis, std::vector<size_t> {1});

}

};
#endif
