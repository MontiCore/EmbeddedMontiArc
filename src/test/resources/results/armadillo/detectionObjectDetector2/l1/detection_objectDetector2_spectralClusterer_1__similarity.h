/* (c) https://github.com/MontiCore/monticore */
#ifndef DETECTION_OBJECTDETECTOR2_SPECTRALCLUSTERER_1__SIMILARITY
#define DETECTION_OBJECTDETECTOR2_SPECTRALCLUSTERER_1__SIMILARITY
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "HelperA.h"
using namespace arma;
class detection_objectDetector2_spectralClusterer_1__similarity{
const int n = 50;
public:
mat red;
mat green;
mat blue;
mat similarity;
mat degree;
void init()
{
red=mat(n,n);
green=mat(n,n);
blue=mat(n,n);
similarity=mat(n*n,n*n);
degree=mat(n*n,n*n);
}
void execute()
{
int counter2 = 1;
for( auto j=1;j<=(red.n_rows);++j){
for( auto i=1;i<=(red.n_rows);++i){
int counter1 = 1;
for( auto j2=1;j2<=(red.n_rows);++j2){
for( auto i2=1;i2<=(red.n_rows);++i2){
double dist = (sqrt((red(i-1, j-1)-red(i2-1, j2-1))*(red(i-1, j-1)-red(i2-1, j2-1))+(green(i-1, j-1)-green(i2-1, j2-1))*(green(i-1, j-1)-green(i2-1, j2-1))+(blue(i-1, j-1)-blue(i2-1, j2-1))*(blue(i-1, j-1)-blue(i2-1, j2-1))));
similarity(counter2-1, counter1-1) = (exp(-dist/(2)));
counter1 = counter1+1;
}
}
counter2 = counter2+1;
}
}
for( auto k=1;k<=(similarity.n_rows);++k){
degree(k-1, k-1) = (accu(similarity.row(k-1)));
}
}

};
#endif
