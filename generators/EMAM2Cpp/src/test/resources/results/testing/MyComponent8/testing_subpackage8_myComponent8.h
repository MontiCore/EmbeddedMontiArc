/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE8_MYCOMPONENT8
#define TESTING_SUBPACKAGE8_MYCOMPONENT8
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_subpackage8_myComponent8_c1.h"
#include "testing_subpackage8_myComponent8_c2.h"
class testing_subpackage8_myComponent8{
public:
double in1;
double out1;
testing_subpackage8_myComponent8_c1 c1;
testing_subpackage8_myComponent8_c2 c2;
void init()
{
c1.init();
c2.init();
}
void execute()
{
c1.in1 = in1;
c1.in2 = in1;
c2.in1 = in1;
c1.execute();
c2.in2 = c1.out1;
c2.execute();
out1 = c2.out1;
}

};
#endif
