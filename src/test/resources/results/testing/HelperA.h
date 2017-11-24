#ifndef HELPERA_H
#define HELPERA_H
#define _GLIBCXX_USE_CXX11_ABI 0
#include <iostream>
#include "armadillo.h"
#include <stdarg.h>
#include <initializer_list>
using namespace arma;
class HelperA{
public:
static mat getEigenVectors(mat A){
vec eigenValues;
mat eigenVectors;
eig_sym(eigenValues,eigenVectors,A);
return eigenVectors;
}
static vec getEigenValues(mat A){
vec eigenValues;
mat eigenVectors;
eig_sym(eigenValues,eigenVectors,A);
return eigenValues;
}
static mat getKMeansClusters(mat A, int k){
mat clusters;
kmeans(clusters,A,k,random_subset,10,true);
return clusters;
}

static mat getSqrtMat(mat A){
cx_mat result=sqrtmat(A);
return real(result);
}
};
#endif
