#ifndef BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1__MULTIORRECT
#define BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1__MULTIORRECT
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_system_collisionDetection_rectIntersection_1__multiOrRect{
const int x = 10;
public:
bool boolIn[10];
bool boolOut;
void init()
{
}
void execute()
{
bool flag = false;
for( auto i=1;i<=x;++i){
flag = flag||boolIn[i-1];
}
boolOut = flag;
}

};
#endif
