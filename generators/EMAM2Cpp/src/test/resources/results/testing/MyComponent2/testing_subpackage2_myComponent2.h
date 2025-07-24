/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE2_MYCOMPONENT2
#define TESTING_SUBPACKAGE2_MYCOMPONENT2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "types/testing_subpackage2_MyStruct1.h"
class testing_subpackage2_myComponent2{
public:
testing_subpackage2_MyStruct1 in1;
double out1;
void init()
{
}
void execute()
{
out1 = in1.f1;
}

};
#endif
