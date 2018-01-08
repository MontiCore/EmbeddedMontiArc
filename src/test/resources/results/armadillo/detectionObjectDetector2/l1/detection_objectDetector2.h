#ifndef DETECTION_OBJECTDETECTOR2
#define DETECTION_OBJECTDETECTOR2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "detection_objectDetector2_spectralClusterer_1_.h"
using namespace arma;
class detection_objectDetector2{
public:
mat imgFront;
mat imgBack;
mat clusters[2];
detection_objectDetector2_spectralClusterer_1_ spectralClusterer[2];
void init()
{
imgFront=mat(2500,3);
imgBack=mat(2500,3);
clusters[0]=mat(2500,1);
clusters[1]=mat(2500,1);
spectralClusterer[0].init();
spectralClusterer[1].init();
}
void execute()
{
spectralClusterer[0].imgMatrix = imgFront;
spectralClusterer[1].imgMatrix = imgBack;
spectralClusterer[0].execute();
spectralClusterer[1].execute();
clusters[0] = spectralClusterer[0].clusters;
clusters[1] = spectralClusterer[1].clusters;
}

};
#endif
