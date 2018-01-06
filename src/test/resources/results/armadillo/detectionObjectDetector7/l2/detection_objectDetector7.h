#ifndef DETECTION_OBJECTDETECTOR7
#define DETECTION_OBJECTDETECTOR7
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "detection_objectDetector7_spectralClusterer_1_.h"
#include <thread>
using namespace arma;
class detection_objectDetector7{
public:
mat imgFront;
mat imgBack;
mat imgLeft;
mat imgRight;
mat imgFront2;
mat imgBack2;
mat imgLeft2;
mat clusters[7];
detection_objectDetector7_spectralClusterer_1_ spectralClusterer[7];
void init()
{
imgFront=mat(2500,3);
imgBack=mat(2500,3);
imgLeft=mat(2500,3);
imgRight=mat(2500,3);
imgFront2=mat(2500,3);
imgBack2=mat(2500,3);
imgLeft2=mat(2500,3);
clusters[0]=mat(2500,1);
clusters[1]=mat(2500,1);
clusters[2]=mat(2500,1);
clusters[3]=mat(2500,1);
clusters[4]=mat(2500,1);
clusters[5]=mat(2500,1);
clusters[6]=mat(2500,1);
spectralClusterer[0].init();
spectralClusterer[1].init();
spectralClusterer[2].init();
spectralClusterer[3].init();
spectralClusterer[4].init();
spectralClusterer[5].init();
spectralClusterer[6].init();
}
void execute()
{
spectralClusterer[0].imgMatrix = imgFront;
spectralClusterer[1].imgMatrix = imgRight;
spectralClusterer[2].imgMatrix = imgLeft;
spectralClusterer[3].imgMatrix = imgBack;
spectralClusterer[4].imgMatrix = imgFront2;
spectralClusterer[5].imgMatrix = imgBack2;
spectralClusterer[6].imgMatrix = imgLeft2;
std::thread thread1( [ this ] {this->spectralClusterer[0].execute();});
std::thread thread2( [ this ] {this->spectralClusterer[1].execute();});
std::thread thread3( [ this ] {this->spectralClusterer[2].execute();});
std::thread thread4( [ this ] {this->spectralClusterer[3].execute();});
std::thread thread5( [ this ] {this->spectralClusterer[4].execute();});
std::thread thread6( [ this ] {this->spectralClusterer[5].execute();});
std::thread thread7( [ this ] {this->spectralClusterer[6].execute();});
thread1.join();
thread2.join();
thread3.join();
thread4.join();
thread5.join();
thread6.join();
thread7.join();
clusters[0] = spectralClusterer[0].clusters;
clusters[1] = spectralClusterer[1].clusters;
clusters[2] = spectralClusterer[2].clusters;
clusters[3] = spectralClusterer[3].clusters;
clusters[4] = spectralClusterer[4].clusters;
clusters[5] = spectralClusterer[5].clusters;
clusters[6] = spectralClusterer[6].clusters;
}

};
#endif
