/* (c) https://github.com/MontiCore/monticore */
#ifndef DETECTION_OBJECTDETECTOR2
#define DETECTION_OBJECTDETECTOR2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
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
spectralClusterer.similarity.red = red1;
spectralClusterer.similarity.green = green1;
spectralClusterer.similarity.blue = blue1;
spectralClusterer.similarity.red = red2;
spectralClusterer.similarity.green = green2;
spectralClusterer.similarity.blue = blue2;
std::thread thread1( [ this ] {this->spectralClusterer[0].similarity.execute();});
std::thread thread2( [ this ] {this->spectralClusterer[1].similarity.execute();});
thread1.join();
thread2.join();
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
std::thread thread3( [ this ] {this->spectralClusterer[0].normalizedLaplacian.execute();});
std::thread thread4( [ this ] {this->spectralClusterer[1].normalizedLaplacian.execute();});
thread3.join();
thread4.join();
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
std::thread thread5( [ this ] {this->spectralClusterer[0].eigenSolver.execute();});
std::thread thread6( [ this ] {this->spectralClusterer[1].eigenSolver.execute();});
thread5.join();
thread6.join();
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
std::thread thread7( [ this ] {this->spectralClusterer[0].kMeansClustering.execute();});
std::thread thread8( [ this ] {this->spectralClusterer[1].kMeansClustering.execute();});
thread7.join();
thread8.join();
clusters[0] = spectralClusterer.kMeansClustering.clusters;
clusters[1] = spectralClusterer.kMeansClustering.clusters;
}

};
#endif
