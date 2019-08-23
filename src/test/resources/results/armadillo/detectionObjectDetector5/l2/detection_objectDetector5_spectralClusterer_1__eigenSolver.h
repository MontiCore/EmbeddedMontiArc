/* (c) https://github.com/MontiCore/monticore */
#ifndef DETECTION_OBJECTDETECTOR5_SPECTRALCLUSTERER_1__EIGENSOLVER
#define DETECTION_OBJECTDETECTOR5_SPECTRALCLUSTERER_1__EIGENSOLVER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "HelperA.h"
#include <thread>
using namespace arma;
class detection_objectDetector5_spectralClusterer_1__eigenSolver{
const int n = 2500;
const int targetEigenvectors = 4;
public:
mat matrix;
mat eigenvectors;
void init()
{
matrix=mat(n,n);
eigenvectors=mat(n,targetEigenvectors);
}
void execute()
{
mat eigenVectors = (HelperA::getEigenVectors((matrix)));
double counter = 1;
double start = (eigenVectors.n_cols)-(targetEigenvectors-1);
for( auto i=start;i<=(eigenVectors.n_rows);++i){
eigenvectors.col(counter-1) = eigenVectors.col(i-1);
counter = counter+1;
}
}

};
#endif
