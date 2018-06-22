#ifndef BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1__DUALSETCOMPARELINES
#define BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1__DUALSETCOMPARELINES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_system_collisionDetection_rectIntersection_1__dualSetCompareLines{
const int rows = 2;
const int cols = 4;
const int n = 4;
const int n2 = 10;
public:
mat setInA[4];
mat setInB[4];
mat outA[10];
mat outB[10];
void init()
{
setInA[0]=mat(rows,cols);
setInA[1]=mat(rows,cols);
setInA[2]=mat(rows,cols);
setInA[3]=mat(rows,cols);
setInB[0]=mat(rows,cols);
setInB[1]=mat(rows,cols);
setInB[2]=mat(rows,cols);
setInB[3]=mat(rows,cols);
outA[0]=mat(rows,cols);
outA[1]=mat(rows,cols);
outA[2]=mat(rows,cols);
outA[3]=mat(rows,cols);
outA[4]=mat(rows,cols);
outA[5]=mat(rows,cols);
outA[6]=mat(rows,cols);
outA[7]=mat(rows,cols);
outA[8]=mat(rows,cols);
outA[9]=mat(rows,cols);
outB[0]=mat(rows,cols);
outB[1]=mat(rows,cols);
outB[2]=mat(rows,cols);
outB[3]=mat(rows,cols);
outB[4]=mat(rows,cols);
outB[5]=mat(rows,cols);
outB[6]=mat(rows,cols);
outB[7]=mat(rows,cols);
outB[8]=mat(rows,cols);
outB[9]=mat(rows,cols);
}
void execute()
{
int counter = 1;
for( auto i=1;i<=n;++i){
for( auto j=i;j<=n;++j){
outA[counter-1] = setInA[i-1];
outB[counter-1] = setInB[j-1];
counter = counter+1;
}
}
}

};
#endif
