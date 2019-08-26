/* (c) https://github.com/MontiCore/monticore */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "test_math_sumCommandTest.h"
int main(int argc, char** argv)
{
Helper::init();
test_math_sumCommandTest testInstance;
testInstance.init();
testInstance.in1=0;testInstance.execute();
if(testInstance.out1>(6-1)&& testInstance.out1<(6+1)){printf("Mismatch at executionStep 0");
octave_quit();
}
testInstance.in1=1;testInstance.execute();
if(testInstance.out1!=7){printf("Mismatch at executionStep 1");
octave_quit();
}
testInstance.in1=2;testInstance.execute();
if(testInstance.out1!=8){printf("Mismatch at executionStep 2");
octave_quit();
}
octave_quit();
printf("Execution ended successfully!\n");
}
