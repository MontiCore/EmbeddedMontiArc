/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_SINGLEELEMARRAY
#define TEST_SINGLEELEMARRAY
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "test_singleElemArray_arrayComp_1_.h"
class test_singleElemArray{
public:
double arrayPort[1];
test_singleElemArray_arrayComp_1_ arrayComp[1];
void init()
{
arrayComp[0].init();
}
void execute()
{
arrayComp[0].execute();
}

};
#endif
