/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_BASICGENERICINSTANCE_BASICGENERIC
#define TEST_BASICGENERICINSTANCE_BASICGENERIC
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class test_basicGenericInstance_basicGeneric{
const int n = 3;
public:
Matrix mat1;
Matrix matOut;
void init()
{
mat1=Matrix(n,n);
matOut=Matrix(n,n);
}
void execute()
{
matOut = mat1*2;
}

};
#endif
