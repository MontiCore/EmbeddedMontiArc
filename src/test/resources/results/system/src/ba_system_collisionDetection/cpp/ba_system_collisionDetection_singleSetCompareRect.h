#ifndef BA_SYSTEM_COLLISIONDETECTION_SINGLESETCOMPARERECT
#define BA_SYSTEM_COLLISIONDETECTION_SINGLESETCOMPARERECT
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "types/ba_struct_Rectangle.h"
using namespace arma;
class ba_system_collisionDetection_singleSetCompareRect{
const int n = 2;
const int x = 1;
public:
ba_struct_Rectangle setIn[2];
ba_struct_Rectangle outA[1];
ba_struct_Rectangle outB[1];
void init()
{
}
void execute()
{
int counter = 1;
for( auto i=1;i<=n;++i){
for( auto j=(i+1);j<=n;++j){
outA[counter-1] = setIn[i-1];
outB[counter-1] = setIn[j-1];
counter = counter+1;
}
}
}

};
#endif
