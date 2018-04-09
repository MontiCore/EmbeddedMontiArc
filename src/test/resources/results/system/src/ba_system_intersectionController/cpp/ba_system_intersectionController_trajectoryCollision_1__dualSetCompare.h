#ifndef BA_SYSTEM_INTERSECTIONCONTROLLER_TRAJECTORYCOLLISION_1__DUALSETCOMPARE
#define BA_SYSTEM_INTERSECTIONCONTROLLER_TRAJECTORYCOLLISION_1__DUALSETCOMPARE
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_system_intersectionController_trajectoryCollision_1__dualSetCompare{
const int rows = 4;
const int cols = 1;
const int n = 9;
const int n2 = 45;
public:
mat setInA[9];
mat setInB[9];
mat outA[45];
mat outB[45];
void init()
{
setInA[0]=mat(rows,cols);
setInA[1]=mat(rows,cols);
setInA[2]=mat(rows,cols);
setInA[3]=mat(rows,cols);
setInA[4]=mat(rows,cols);
setInA[5]=mat(rows,cols);
setInA[6]=mat(rows,cols);
setInA[7]=mat(rows,cols);
setInA[8]=mat(rows,cols);
setInB[0]=mat(rows,cols);
setInB[1]=mat(rows,cols);
setInB[2]=mat(rows,cols);
setInB[3]=mat(rows,cols);
setInB[4]=mat(rows,cols);
setInB[5]=mat(rows,cols);
setInB[6]=mat(rows,cols);
setInB[7]=mat(rows,cols);
setInB[8]=mat(rows,cols);
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
outA[10]=mat(rows,cols);
outA[11]=mat(rows,cols);
outA[12]=mat(rows,cols);
outA[13]=mat(rows,cols);
outA[14]=mat(rows,cols);
outA[15]=mat(rows,cols);
outA[16]=mat(rows,cols);
outA[17]=mat(rows,cols);
outA[18]=mat(rows,cols);
outA[19]=mat(rows,cols);
outA[20]=mat(rows,cols);
outA[21]=mat(rows,cols);
outA[22]=mat(rows,cols);
outA[23]=mat(rows,cols);
outA[24]=mat(rows,cols);
outA[25]=mat(rows,cols);
outA[26]=mat(rows,cols);
outA[27]=mat(rows,cols);
outA[28]=mat(rows,cols);
outA[29]=mat(rows,cols);
outA[30]=mat(rows,cols);
outA[31]=mat(rows,cols);
outA[32]=mat(rows,cols);
outA[33]=mat(rows,cols);
outA[34]=mat(rows,cols);
outA[35]=mat(rows,cols);
outA[36]=mat(rows,cols);
outA[37]=mat(rows,cols);
outA[38]=mat(rows,cols);
outA[39]=mat(rows,cols);
outA[40]=mat(rows,cols);
outA[41]=mat(rows,cols);
outA[42]=mat(rows,cols);
outA[43]=mat(rows,cols);
outA[44]=mat(rows,cols);
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
outB[10]=mat(rows,cols);
outB[11]=mat(rows,cols);
outB[12]=mat(rows,cols);
outB[13]=mat(rows,cols);
outB[14]=mat(rows,cols);
outB[15]=mat(rows,cols);
outB[16]=mat(rows,cols);
outB[17]=mat(rows,cols);
outB[18]=mat(rows,cols);
outB[19]=mat(rows,cols);
outB[20]=mat(rows,cols);
outB[21]=mat(rows,cols);
outB[22]=mat(rows,cols);
outB[23]=mat(rows,cols);
outB[24]=mat(rows,cols);
outB[25]=mat(rows,cols);
outB[26]=mat(rows,cols);
outB[27]=mat(rows,cols);
outB[28]=mat(rows,cols);
outB[29]=mat(rows,cols);
outB[30]=mat(rows,cols);
outB[31]=mat(rows,cols);
outB[32]=mat(rows,cols);
outB[33]=mat(rows,cols);
outB[34]=mat(rows,cols);
outB[35]=mat(rows,cols);
outB[36]=mat(rows,cols);
outB[37]=mat(rows,cols);
outB[38]=mat(rows,cols);
outB[39]=mat(rows,cols);
outB[40]=mat(rows,cols);
outB[41]=mat(rows,cols);
outB[42]=mat(rows,cols);
outB[43]=mat(rows,cols);
outB[44]=mat(rows,cols);
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
