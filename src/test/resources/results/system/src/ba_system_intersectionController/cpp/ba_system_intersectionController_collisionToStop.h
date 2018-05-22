#ifndef BA_SYSTEM_INTERSECTIONCONTROLLER_COLLISIONTOSTOP
#define BA_SYSTEM_INTERSECTIONCONTROLLER_COLLISIONTOSTOP
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_system_intersectionController_collisionToStop{
const int n = 2;
const int x = 1;
public:
bool collisionIn[1];
bool aIsFasterIn[1];
bool active;
bool stopOut[2];
void init()
{
}
void execute()
{
ivec counter=ivec(n);
ivec indexLookup=ivec(x);
for( auto i=1;i<=n;++i){
counter(i-1) = 0;
}
int k = 1;
int maxI = n-1;
for( auto i=1;i<=maxI;++i){
int minJ = i+1;
for( auto j=minJ;j<=n;++j){
counter(i-1) = counter(i-1)+1;
counter(j-1) = counter(j-1)+1;
if(aIsFasterIn[k-1]){
indexLookup(k-1) = j;
}
else {
indexLookup(k-1) = i;
}
k = k+1;
}
}
for( auto i=1;i<=n;++i){
stopOut[i-1] = false;
}
if(active){
for( auto i=1;i<=x;++i){
if(collisionIn[i-1]){
int curIndex = indexLookup(i-1);
stopOut[curIndex-1] = true;
}
}
}
}

};
#endif
