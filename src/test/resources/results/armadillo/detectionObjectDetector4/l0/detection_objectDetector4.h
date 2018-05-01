#ifndef DETECTION_OBJECTDETECTOR4
#define DETECTION_OBJECTDETECTOR4
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "detection_objectDetector4_spectralClusterer_1_.h"
using namespace arma;
class detection_objectDetector4{
public:
mat red1;
mat green1;
mat blue1;
mat red2;
mat green2;
mat blue2;
mat red3;
mat green3;
mat blue3;
mat red4;
mat green4;
mat blue4;
mat clusters[4];
detection_objectDetector4_spectralClusterer_1_ spectralClusterer[4];
void init()
{
red1=mat(50,50);
green1=mat(50,50);
blue1=mat(50,50);
red2=mat(50,50);
green2=mat(50,50);
blue2=mat(50,50);
red3=mat(50,50);
green3=mat(50,50);
blue3=mat(50,50);
red4=mat(50,50);
green4=mat(50,50);
blue4=mat(50,50);
clusters[0]=mat(2500,1);
clusters[1]=mat(2500,1);
clusters[2]=mat(2500,1);
clusters[3]=mat(2500,1);
spectralClusterer[0].init();
spectralClusterer[1].init();
spectralClusterer[2].init();
spectralClusterer[3].init();
}
void execute()
{
spectralClusterer[0].red = red1;
spectralClusterer[0].green = green1;
spectralClusterer[0].blue = blue1;
spectralClusterer[0].execute();
spectralClusterer[1].red = red2;
spectralClusterer[1].green = green2;
spectralClusterer[1].blue = blue2;
spectralClusterer[1].execute();
spectralClusterer[2].red = red3;
spectralClusterer[2].green = green3;
spectralClusterer[2].blue = blue3;
spectralClusterer[2].execute();
spectralClusterer[3].red = red4;
spectralClusterer[3].green = green4;
spectralClusterer[3].blue = blue4;
spectralClusterer[3].execute();
clusters[0] = spectralClusterer[0].clusters;
clusters[1] = spectralClusterer[1].clusters;
clusters[2] = spectralClusterer[2].clusters;
clusters[3] = spectralClusterer[3].clusters;
}

};
#endif
