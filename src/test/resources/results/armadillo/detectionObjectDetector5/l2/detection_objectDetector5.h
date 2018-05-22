#ifndef DETECTION_OBJECTDETECTOR5
#define DETECTION_OBJECTDETECTOR5
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "detection_objectDetector5_spectralClusterer_1_.h"
#include <thread>
using namespace arma;
class detection_objectDetector5{
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
mat red5;
mat green5;
mat blue5;
mat clusters[5];
detection_objectDetector5_spectralClusterer_1_ spectralClusterer[5];
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
red5=mat(50,50);
green5=mat(50,50);
blue5=mat(50,50);
clusters[0]=mat(2500,1);
clusters[1]=mat(2500,1);
clusters[2]=mat(2500,1);
clusters[3]=mat(2500,1);
clusters[4]=mat(2500,1);
spectralClusterer[0].init();
spectralClusterer[1].init();
spectralClusterer[2].init();
spectralClusterer[3].init();
spectralClusterer[4].init();
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
spectralClusterer[2].red = red3;
spectralClusterer[2].green = green3;
spectralClusterer[2].blue = blue3;
std::thread thread3( [ this ] {this->spectralClusterer[2].execute();});
spectralClusterer[3].red = red4;
spectralClusterer[3].green = green4;
spectralClusterer[3].blue = blue4;
std::thread thread4( [ this ] {this->spectralClusterer[3].execute();});
spectralClusterer[4].red = red5;
spectralClusterer[4].green = green5;
spectralClusterer[4].blue = blue5;
std::thread thread5( [ this ] {this->spectralClusterer[4].execute();});
thread1.join();
thread2.join();
thread3.join();
thread4.join();
thread5.join();
clusters[0] = spectralClusterer[0].clusters;
clusters[1] = spectralClusterer[1].clusters;
clusters[2] = spectralClusterer[2].clusters;
clusters[3] = spectralClusterer[3].clusters;
clusters[4] = spectralClusterer[4].clusters;
}

};
#endif
