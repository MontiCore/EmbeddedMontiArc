/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_BASICLOOKUPINSTANCE_LOOKUP2
#define TESTING_BASICLOOKUPINSTANCE_LOOKUP2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class testing_basicLookUpInstance_lookUp2{
const int n = 4;
public:
Matrix lookuptable;
double in1;
double out1;
void init(Matrix lookuptable)
{
this->lookuptable=lookuptable;
}
void execute()
{
out1 = lookuptable(in1);
}

};
#endif
