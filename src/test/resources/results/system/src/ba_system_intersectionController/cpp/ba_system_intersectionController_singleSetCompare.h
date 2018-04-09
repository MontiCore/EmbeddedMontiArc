#ifndef BA_SYSTEM_INTERSECTIONCONTROLLER_SINGLESETCOMPARE
#define BA_SYSTEM_INTERSECTIONCONTROLLER_SINGLESETCOMPARE
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_system_intersectionController_singleSetCompare{
const int rows = 3;
const int cols = 10;
const int n = 2;
const int x = 1;
public:
mat setIn[2];
mat outA[1];
mat outB[1];
void init()
{
setIn[0]=mat(rows,cols);
setIn[1]=mat(rows,cols);
outA[0]=mat(rows,cols);
outB[0]=mat(rows,cols);
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
