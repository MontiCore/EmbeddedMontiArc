#ifndef DETECTION_OBJECTDETECTOR1
#define DETECTION_OBJECTDETECTOR1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "detection_objectDetector1_spectralClusterer.h"
#include <thread>
using namespace arma;
class detection_objectDetector1{
public:
mat red1;
mat green1;
mat blue1;
mat clusters;
detection_objectDetector1_spectralClusterer spectralClusterer;
void init()
{
red1=mat(50,50);
green1=mat(50,50);
blue1=mat(50,50);
clusters=mat(2500,1);
spectralClusterer.init();
}
void execute()
{
spectralClusterer.red = red1;
spectralClusterer.green = green1;
spectralClusterer.blue = blue1;
std::thread thread1( [ this ] {this->spectralClusterer.execute();});
thread1.join();
clusters = spectralClusterer.clusters;
}

};
#endif
