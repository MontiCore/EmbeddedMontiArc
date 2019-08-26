/* (c) https://github.com/MontiCore/monticore */
#ifndef DETECTION_OBJECTDETECTOR8_SPECTRALCLUSTERER_1__KMEANSCLUSTERING
#define DETECTION_OBJECTDETECTOR8_SPECTRALCLUSTERER_1__KMEANSCLUSTERING
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "HelperA.h"
using namespace arma;
class detection_objectDetector8_spectralClusterer_1__kMeansClustering{
const int n = 2500;
const int amountVectors = 4;
const int maximumClusters = 1;
public:
mat vectors;
mat clusters;
void init()
{
vectors=mat(n,amountVectors);
clusters=mat(n,1);
}
void execute()
{
mat UMatrix=mat(n,amountVectors);
for( auto i=1;i<=(vectors.n_rows);++i){
rowvec target = pow(vectors.row(i-1),2);
double amount = (sqrt((accu(target))));
UMatrix.row(i-1) = vectors.row(i-1)/amount;
}
clusters = (HelperA::getKMeansClusters(UMatrix, maximumClusters));
}

};
#endif
