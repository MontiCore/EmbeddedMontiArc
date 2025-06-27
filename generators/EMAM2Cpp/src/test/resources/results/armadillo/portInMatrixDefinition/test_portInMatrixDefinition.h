/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_PORTINMATRIXDEFINITION
#define TEST_PORTINMATRIXDEFINITION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "HelperA.h"
using namespace arma;
class test_portInMatrixDefinition{
public:
double in1;
double out1;
void init()
{
}
void execute()
{
colvec a = (rowvec({0,in1,1}).t());
a = a+1;
rowvec b = rowvec({a(1-1),a(2-1),a(3-1)});
mat c = mat({{a(1-1),a(2-1)},{b(1-1),b(2-1)}});
out1 = (accu(c));
}

};
#endif
