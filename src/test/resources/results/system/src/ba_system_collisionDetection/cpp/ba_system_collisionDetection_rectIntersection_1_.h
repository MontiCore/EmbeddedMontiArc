#ifndef BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1_
#define BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1_
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "types/ba_struct_Rectangle.h"
#include "ba_system_collisionDetection_rectIntersection_1__rectToLines1.h"
#include "ba_system_collisionDetection_rectIntersection_1__rectToLines2.h"
#include "ba_system_collisionDetection_rectIntersection_1__dualSetCompareLines.h"
#include "ba_system_collisionDetection_rectIntersection_1__lineIntersection_1_.h"
#include "ba_system_collisionDetection_rectIntersection_1__multiOrRect.h"
using namespace arma;
class ba_system_collisionDetection_rectIntersection_1_{
public:
ba_struct_Rectangle rect1;
ba_struct_Rectangle rect2;
bool collision;
ba_system_collisionDetection_rectIntersection_1__rectToLines1 rectToLines1;
ba_system_collisionDetection_rectIntersection_1__rectToLines2 rectToLines2;
ba_system_collisionDetection_rectIntersection_1__dualSetCompareLines dualSetCompareLines;
ba_system_collisionDetection_rectIntersection_1__lineIntersection_1_ lineIntersection[10];
ba_system_collisionDetection_rectIntersection_1__multiOrRect multiOrRect;
void init()
{
rectToLines1.init();
rectToLines2.init();
dualSetCompareLines.init();
lineIntersection[0].init();
lineIntersection[1].init();
lineIntersection[2].init();
lineIntersection[3].init();
lineIntersection[4].init();
lineIntersection[5].init();
lineIntersection[6].init();
lineIntersection[7].init();
lineIntersection[8].init();
lineIntersection[9].init();
multiOrRect.init();
}
void execute()
{
rectToLines1.rect = rect1;
rectToLines1.execute();
rectToLines2.rect = rect2;
rectToLines2.execute();
dualSetCompareLines.setInA[0] = rectToLines1.lines[0];
dualSetCompareLines.setInA[1] = rectToLines1.lines[1];
dualSetCompareLines.setInA[2] = rectToLines1.lines[2];
dualSetCompareLines.setInA[3] = rectToLines1.lines[3];
dualSetCompareLines.setInB[0] = rectToLines2.lines[0];
dualSetCompareLines.setInB[1] = rectToLines2.lines[1];
dualSetCompareLines.setInB[2] = rectToLines2.lines[2];
dualSetCompareLines.setInB[3] = rectToLines2.lines[3];
dualSetCompareLines.execute();
lineIntersection[0].lineA = dualSetCompareLines.outA[0];
lineIntersection[0].lineB = dualSetCompareLines.outB[0];
lineIntersection[0].execute();
lineIntersection[1].lineA = dualSetCompareLines.outA[1];
lineIntersection[1].lineB = dualSetCompareLines.outB[1];
lineIntersection[1].execute();
lineIntersection[2].lineA = dualSetCompareLines.outA[2];
lineIntersection[2].lineB = dualSetCompareLines.outB[2];
lineIntersection[2].execute();
lineIntersection[3].lineA = dualSetCompareLines.outA[3];
lineIntersection[3].lineB = dualSetCompareLines.outB[3];
lineIntersection[3].execute();
lineIntersection[4].lineA = dualSetCompareLines.outA[4];
lineIntersection[4].lineB = dualSetCompareLines.outB[4];
lineIntersection[4].execute();
lineIntersection[5].lineA = dualSetCompareLines.outA[5];
lineIntersection[5].lineB = dualSetCompareLines.outB[5];
lineIntersection[5].execute();
lineIntersection[6].lineA = dualSetCompareLines.outA[6];
lineIntersection[6].lineB = dualSetCompareLines.outB[6];
lineIntersection[6].execute();
lineIntersection[7].lineA = dualSetCompareLines.outA[7];
lineIntersection[7].lineB = dualSetCompareLines.outB[7];
lineIntersection[7].execute();
lineIntersection[8].lineA = dualSetCompareLines.outA[8];
lineIntersection[8].lineB = dualSetCompareLines.outB[8];
lineIntersection[8].execute();
lineIntersection[9].lineA = dualSetCompareLines.outA[9];
lineIntersection[9].lineB = dualSetCompareLines.outB[9];
lineIntersection[9].execute();
multiOrRect.boolIn[0] = lineIntersection[0].intersects;
multiOrRect.boolIn[1] = lineIntersection[1].intersects;
multiOrRect.boolIn[2] = lineIntersection[2].intersects;
multiOrRect.boolIn[3] = lineIntersection[3].intersects;
multiOrRect.boolIn[4] = lineIntersection[4].intersects;
multiOrRect.boolIn[5] = lineIntersection[5].intersects;
multiOrRect.boolIn[6] = lineIntersection[6].intersects;
multiOrRect.boolIn[7] = lineIntersection[7].intersects;
multiOrRect.boolIn[8] = lineIntersection[8].intersects;
multiOrRect.boolIn[9] = lineIntersection[9].intersects;
multiOrRect.execute();
collision = multiOrRect.boolOut;
}

};
#endif
