#ifndef DETECTION_OBJECTDETECTOR8_SPECTRALCLUSTERER_1__SIMILARITY
#define DETECTION_OBJECTDETECTOR8_SPECTRALCLUSTERER_1__SIMILARITY
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "HelperA.h"
#include <thread>
using namespace arma;
class detection_objectDetector8_spectralClusterer_1__similarity{
const int n = 2500;
public:
mat data;
mat similarity;
mat degree;
void init()
{
data=mat(n,3);
similarity=mat(n,n);
degree=mat(n,n);
}
void execute()
{
for( auto i=1;i<=(data.n_rows);++i){
for( auto j=1;j<=(data.n_rows);++j){
double dist = (sqrt((data(i-1, 1-1)-data(j-1, 1-1))*(data(i-1, 1-1)-data(j-1, 1-1))+(data(i-1, 2-1)-data(j-1, 2-1))*(data(i-1, 2-1)-data(j-1, 2-1))+(data(i-1, 3-1)-data(j-1, 3-1))*(data(i-1, 3-1)-data(j-1, 3-1))));
similarity(i-1, j-1) = (exp((0-dist)/(2)));
}
}
for( auto k=1;k<=(similarity.n_rows);++k){
degree(k-1, k-1) = (accu(similarity.row(k-1)));
}
}

};
#endif
