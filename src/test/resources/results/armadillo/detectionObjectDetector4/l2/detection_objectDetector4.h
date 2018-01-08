#ifndef DETECTION_OBJECTDETECTOR4
#define DETECTION_OBJECTDETECTOR4
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "detection_objectDetector4_spectralClusterer_1_.h"
#include <thread>
using namespace arma;
class detection_objectDetector4{
public:
mat imgFront;
mat imgBack;
mat imgLeft;
mat imgRight;
mat clusters[4];
detection_objectDetector4_spectralClusterer_1_ spectralClusterer[4];
void init()
{
imgFront=mat(2500,3);
imgBack=mat(2500,3);
imgLeft=mat(2500,3);
imgRight=mat(2500,3);
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
spectralClusterer[0].imgMatrix = imgFront;
spectralClusterer[1].imgMatrix = imgRight;
spectralClusterer[2].imgMatrix = imgLeft;
spectralClusterer[3].imgMatrix = imgBack;
std::thread thread1( [ this ] {this->spectralClusterer[0].execute();});
std::thread thread2( [ this ] {this->spectralClusterer[1].execute();});
std::thread thread3( [ this ] {this->spectralClusterer[2].execute();});
std::thread thread4( [ this ] {this->spectralClusterer[3].execute();});
thread1.join();
thread2.join();
thread3.join();
thread4.join();
clusters[0] = spectralClusterer[0].clusters;
clusters[1] = spectralClusterer[1].clusters;
clusters[2] = spectralClusterer[2].clusters;
clusters[3] = spectralClusterer[3].clusters;
}

};
#endif
