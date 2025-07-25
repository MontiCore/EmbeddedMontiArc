/* (c) https://github.com/MontiCore/monticore */
#ifndef TEST_BASICPORTSMATH
#define TEST_BASICPORTSMATH
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class test_basicPortsMath{
public:
double counter;
double result;
void init()
{
}
void execute()
{
if((counter < 0)){
result = 0;
}
else if((counter < 100)){
result = counter;
}
else {
result = 100;
}
}

};
#endif
