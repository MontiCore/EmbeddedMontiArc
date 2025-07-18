/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_BASICGENERICINSTANCE
#define TEST_BASICGENERICINSTANCE
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "test_basicGenericInstance_basicGeneric.h"
class test_basicGenericInstance{
public:
Matrix mat1;
Matrix matOut;
test_basicGenericInstance_basicGeneric basicGeneric;
void init()
{
mat1=Matrix(3,3);
matOut=Matrix(3,3);
basicGeneric.init();
}
void execute()
{
basicGeneric.mat1 = mat1;
basicGeneric.execute();
matOut = basicGeneric.matOut;
}

};
#endif
