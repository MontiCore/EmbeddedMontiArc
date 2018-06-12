#ifndef DETECTION_OBJECTDETECTOR9
#define DETECTION_OBJECTDETECTOR9
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "detection_objectDetector9_spectralClusterer_1_.h"
#include <thread>
using namespace arma;
class detection_objectDetector9{
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
mat red6;
mat green6;
mat blue6;
mat red7;
mat green7;
mat blue7;
mat red8;
mat green8;
mat blue8;
mat red9;
mat green9;
mat blue9;
mat clusters[9];
detection_objectDetector9_spectralClusterer_1_ spectralClusterer[9];
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
red6=mat(50,50);
green6=mat(50,50);
blue6=mat(50,50);
red7=mat(50,50);
green7=mat(50,50);
blue7=mat(50,50);
red8=mat(50,50);
green8=mat(50,50);
blue8=mat(50,50);
red9=mat(50,50);
green9=mat(50,50);
blue9=mat(50,50);
clusters[0]=mat(2500,1);
clusters[1]=mat(2500,1);
clusters[2]=mat(2500,1);
clusters[3]=mat(2500,1);
clusters[4]=mat(2500,1);
clusters[5]=mat(2500,1);
clusters[6]=mat(2500,1);
clusters[7]=mat(2500,1);
clusters[8]=mat(2500,1);
spectralClusterer[0].init();
spectralClusterer[1].init();
spectralClusterer[2].init();
spectralClusterer[3].init();
spectralClusterer[4].init();
spectralClusterer[5].init();
spectralClusterer[6].init();
spectralClusterer[7].init();
spectralClusterer[8].init();
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
spectralClusterer[5].red = red6;
spectralClusterer[5].green = green6;
spectralClusterer[5].blue = blue6;
std::thread thread6( [ this ] {this->spectralClusterer[5].execute();});
spectralClusterer[6].red = red7;
spectralClusterer[6].green = green7;
spectralClusterer[6].blue = blue7;
std::thread thread7( [ this ] {this->spectralClusterer[6].execute();});
spectralClusterer[7].red = red8;
spectralClusterer[7].green = green8;
spectralClusterer[7].blue = blue8;
std::thread thread8( [ this ] {this->spectralClusterer[7].execute();});
spectralClusterer[8].red = red9;
spectralClusterer[8].green = green9;
spectralClusterer[8].blue = blue9;
std::thread thread9( [ this ] {this->spectralClusterer[8].execute();});
thread1.join();
thread2.join();
thread3.join();
thread4.join();
thread5.join();
thread6.join();
thread7.join();
thread8.join();
thread9.join();
clusters[0] = spectralClusterer[0].clusters;
clusters[1] = spectralClusterer[1].clusters;
clusters[2] = spectralClusterer[2].clusters;
clusters[3] = spectralClusterer[3].clusters;
clusters[4] = spectralClusterer[4].clusters;
clusters[5] = spectralClusterer[5].clusters;
clusters[6] = spectralClusterer[6].clusters;
clusters[7] = spectralClusterer[7].clusters;
clusters[8] = spectralClusterer[8].clusters;
}

};
#endif
