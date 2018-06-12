#ifndef DETECTION_OBJECTDETECTOR2
#define DETECTION_OBJECTDETECTOR2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "detection_objectDetector2_spectralClusterer_1_.h"
#include <thread>
using namespace arma;
class detection_objectDetector2{
public:
mat red1;
mat green1;
mat blue1;
mat red2;
mat green2;
mat blue2;
mat clusters[2];
detection_objectDetector2_spectralClusterer_1_ spectralClusterer[2];
void init()
{
red1=mat(50,50);
green1=mat(50,50);
blue1=mat(50,50);
red2=mat(50,50);
green2=mat(50,50);
blue2=mat(50,50);
clusters[0]=mat(2500,1);
clusters[1]=mat(2500,1);
spectralClusterer[0].init();
spectralClusterer[1].init();
}
void execute()
{
spectralClusterer[0].red = red1;
spectralClusterer[0].green = green1;
spectralClusterer[0].blue = blue1;
std::thread thread1( [ this ] {this->spectralClusterer[0].execute();});
spectralClusterer[1].red = red2;
spectralClusterer[1].green = green2;
spectralClusterer[1].blue = blue2;
std::thread thread2( [ this ] {this->spectralClusterer[1].execute();});
thread1.join();
thread2.join();
clusters[0] = spectralClusterer[0].clusters;
clusters[1] = spectralClusterer[1].clusters;
}

};
#endif
