#ifndef BA_SYSTEM_COLLISIONDETECTION
#define BA_SYSTEM_COLLISIONDETECTION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "types/ba_struct_Rectangle.h"
#include "ba_system_collisionDetection_singleSetCompareRect.h"
#include "ba_system_collisionDetection_rectIntersection_1_.h"
#include "ba_system_collisionDetection_multiOr.h"
using namespace arma;
class ba_system_collisionDetection{
const int n = 2;
const int x = 1;
public:
ba_struct_Rectangle hulls[2];
bool collision;
ba_system_collisionDetection_singleSetCompareRect singleSetCompareRect;
ba_system_collisionDetection_rectIntersection_1_ rectIntersection[1];
ba_system_collisionDetection_multiOr multiOr;
void init()
{
singleSetCompareRect.init();
rectIntersection[0].init();
multiOr.init();
}
void execute()
{
singleSetCompareRect.setIn[0] = hulls[0];
singleSetCompareRect.setIn[1] = hulls[1];
singleSetCompareRect.execute();
rectIntersection[0].rect1 = singleSetCompareRect.outA[0];
rectIntersection[0].rect2 = singleSetCompareRect.outB[0];
rectIntersection[0].execute();
multiOr.boolIn[0] = rectIntersection[0].collision;
multiOr.execute();
collision = multiOr.boolOut;
}

};
#endif
