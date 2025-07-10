/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_BASICGENERICARRAYINSTANCE
#define TEST_BASICGENERICARRAYINSTANCE
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "test_basicGenericArrayInstance_basicGenericArraySize1.h"
#include "test_basicGenericArrayInstance_basicGenericArraySize2.h"
class test_basicGenericArrayInstance{
public:
double val1[6];
double valOut[6];
test_basicGenericArrayInstance_basicGenericArraySize1 basicGenericArraySize1;
test_basicGenericArrayInstance_basicGenericArraySize2 basicGenericArraySize2;
void init()
{
basicGenericArraySize1.init();
basicGenericArraySize2.init();
}
void execute()
{
basicGenericArraySize1.val1[0] = val1[0];
basicGenericArraySize1.val1[1] = val1[1];
basicGenericArraySize1.val1[2] = val1[2];
basicGenericArraySize2.val1[0] = val1[3];
basicGenericArraySize2.val1[1] = val1[4];
basicGenericArraySize2.val1[2] = val1[5];
basicGenericArraySize1.execute();
valOut[0] = basicGenericArraySize1.valOut[0];
valOut[1] = basicGenericArraySize1.valOut[1];
valOut[2] = basicGenericArraySize1.valOut[2];
basicGenericArraySize2.execute();
valOut[3] = basicGenericArraySize2.valOut[0];
valOut[4] = basicGenericArraySize2.valOut[1];
valOut[5] = basicGenericArraySize2.valOut[2];
}

};
#endif
