#ifndef BA_SYSTEM_COLLISIONDETECTION_MULTIOR
#define BA_SYSTEM_COLLISIONDETECTION_MULTIOR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_system_collisionDetection_multiOr{
const int x = 1;
public:
bool boolIn[1];
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
