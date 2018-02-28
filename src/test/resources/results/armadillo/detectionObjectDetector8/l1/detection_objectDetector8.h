#ifndef DETECTION_OBJECTDETECTOR8
#define DETECTION_OBJECTDETECTOR8
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "detection_objectDetector8_spectralClusterer_1_.h"
using namespace arma;
class detection_objectDetector8{
public:
mat imgFront;
mat imgBack;
mat imgLeft;
mat imgRight;
mat imgFront2;
mat imgBack2;
mat imgLeft2;
mat imgRight2;
mat clusters[8];
detection_objectDetector8_spectralClusterer_1_ spectralClusterer[8];
void init()
{
imgFront=mat(2500,3);
imgBack=mat(2500,3);
imgLeft=mat(2500,3);
imgRight=mat(2500,3);
imgFront2=mat(2500,3);
imgBack2=mat(2500,3);
imgLeft2=mat(2500,3);
imgRight2=mat(2500,3);
clusters[0]=mat(2500,1);
clusters[1]=mat(2500,1);
clusters[2]=mat(2500,1);
clusters[3]=mat(2500,1);
clusters[4]=mat(2500,1);
clusters[5]=mat(2500,1);
clusters[6]=mat(2500,1);
clusters[7]=mat(2500,1);
spectralClusterer[0].init();
spectralClusterer[1].init();
spectralClusterer[2].init();
spectralClusterer[3].init();
spectralClusterer[4].init();
spectralClusterer[5].init();
spectralClusterer[6].init();
spectralClusterer[7].init();
}
void execute()
{
spectralClusterer[0].imgMatrix = imgFront;
spectralClusterer[0].execute();
spectralClusterer[1].imgMatrix = imgRight;
spectralClusterer[1].execute();
spectralClusterer[2].imgMatrix = imgLeft;
spectralClusterer[2].execute();
spectralClusterer[3].imgMatrix = imgBack;
spectralClusterer[3].execute();
spectralClusterer[4].imgMatrix = imgFront2;
spectralClusterer[4].execute();
spectralClusterer[5].imgMatrix = imgRight2;
spectralClusterer[5].execute();
spectralClusterer[6].imgMatrix = imgLeft2;
spectralClusterer[6].execute();
spectralClusterer[7].imgMatrix = imgBack2;
spectralClusterer[7].execute();
clusters[0] = spectralClusterer[0].clusters;
clusters[1] = spectralClusterer[1].clusters;
clusters[2] = spectralClusterer[2].clusters;
clusters[3] = spectralClusterer[3].clusters;
clusters[4] = spectralClusterer[4].clusters;
clusters[5] = spectralClusterer[5].clusters;
clusters[6] = spectralClusterer[6].clusters;
clusters[7] = spectralClusterer[7].clusters;
}

};
#endif
