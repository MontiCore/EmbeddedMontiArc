/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_BASICLOOKUPINSTANCE
#define TESTING_BASICLOOKUPINSTANCE
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_basicLookUpInstance_lookUp1.h"
#include "testing_basicLookUpInstance_lookUp2.h"
#include "testing_basicLookUpInstance_lookUp3.h"
class testing_basicLookUpInstance{
public:
testing_basicLookUpInstance_lookUp1 lookUp1;
testing_basicLookUpInstance_lookUp2 lookUp2;
testing_basicLookUpInstance_lookUp3 lookUp3;
RowVector CONSTANTCONSTANTVECTOR0;
RowVector CONSTANTCONSTANTVECTOR1;
RowVector CONSTANTCONSTANTVECTOR2;
void init()
{
CONSTANTCONSTANTVECTOR0 = RowVector(4);
CONSTANTCONSTANTVECTOR0(0,0) = 0;
CONSTANTCONSTANTVECTOR0(0,1) = 1;
CONSTANTCONSTANTVECTOR0(0,2) = 2;
CONSTANTCONSTANTVECTOR0(0,3) = 3;
lookUp1.init(CONSTANTCONSTANTVECTOR0);
CONSTANTCONSTANTVECTOR1 = RowVector(4);
CONSTANTCONSTANTVECTOR1(0,0) = 0;
CONSTANTCONSTANTVECTOR1(0,1) = 5;
CONSTANTCONSTANTVECTOR1(0,2) = 4;
CONSTANTCONSTANTVECTOR1(0,3) = 4;
lookUp2.init(CONSTANTCONSTANTVECTOR1);
CONSTANTCONSTANTVECTOR2 = RowVector(4);
CONSTANTCONSTANTVECTOR2(0,0) = 1;
CONSTANTCONSTANTVECTOR2(0,1) = 2;
CONSTANTCONSTANTVECTOR2(0,2) = 7;
CONSTANTCONSTANTVECTOR2(0,3) = 9;
lookUp3.init(CONSTANTCONSTANTVECTOR2);
}
void execute()
{
lookUp1.execute();
lookUp2.execute();
lookUp3.execute();
}

};
#endif
