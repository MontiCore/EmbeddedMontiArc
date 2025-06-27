/* (c) https://github.com/MontiCore/monticore */
#ifndef DETECTION_OBJECTDETECTOR1
#define DETECTION_OBJECTDETECTOR1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
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
spectralClusterer.similarity.red = red1;
spectralClusterer.similarity.green = green1;
spectralClusterer.similarity.blue = blue1;
std::thread thread1( [ this ] {this->spectralClusterer.similarity.execute();});
thread1.join();
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
std::thread thread2( [ this ] {this->spectralClusterer.normalizedLaplacian.execute();});
thread2.join();
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
std::thread thread3( [ this ] {this->spectralClusterer.eigenSolver.execute();});
thread3.join();
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
std::thread thread4( [ this ] {this->spectralClusterer.kMeansClustering.execute();});
thread4.join();
clusters = spectralClusterer.kMeansClustering.clusters;
}

};
#endif
