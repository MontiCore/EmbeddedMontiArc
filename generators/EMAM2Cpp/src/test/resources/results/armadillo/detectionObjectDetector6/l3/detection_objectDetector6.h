/* (c) https://github.com/MontiCore/monticore */
#ifndef DETECTION_OBJECTDETECTOR6
#define DETECTION_OBJECTDETECTOR6
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "detection_objectDetector6_spectralClusterer_1_.h"
#include <thread>
using namespace arma;
class detection_objectDetector6{
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
mat clusters[6];
detection_objectDetector6_spectralClusterer_1_ spectralClusterer[6];
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
clusters[0]=mat(2500,1);
clusters[1]=mat(2500,1);
clusters[2]=mat(2500,1);
clusters[3]=mat(2500,1);
clusters[4]=mat(2500,1);
clusters[5]=mat(2500,1);
spectralClusterer[0].init();
spectralClusterer[1].init();
spectralClusterer[2].init();
spectralClusterer[3].init();
spectralClusterer[4].init();
spectralClusterer[5].init();
}
void execute()
{
spectralClusterer[0].similarity.red = red1;
spectralClusterer[0].similarity.green = green1;
spectralClusterer[0].similarity.blue = blue1;
spectralClusterer[1].similarity.red = red2;
spectralClusterer[1].similarity.green = green2;
spectralClusterer[1].similarity.blue = blue2;
spectralClusterer[2].similarity.red = red3;
spectralClusterer[2].similarity.green = green3;
spectralClusterer[2].similarity.blue = blue3;
spectralClusterer[3].similarity.red = red4;
spectralClusterer[3].similarity.green = green4;
spectralClusterer[3].similarity.blue = blue4;
spectralClusterer[4].similarity.red = red5;
spectralClusterer[4].similarity.green = green5;
spectralClusterer[4].similarity.blue = blue5;
spectralClusterer[5].similarity.red = red6;
spectralClusterer[5].similarity.green = green6;
spectralClusterer[5].similarity.blue = blue6;
std::thread thread1( [ this ] {this->spectralClusterer[0].similarity.execute();});
std::thread thread2( [ this ] {this->spectralClusterer[1].similarity.execute();});
std::thread thread3( [ this ] {this->spectralClusterer[2].similarity.execute();});
std::thread thread4( [ this ] {this->spectralClusterer[3].similarity.execute();});
std::thread thread5( [ this ] {this->spectralClusterer[4].similarity.execute();});
std::thread thread6( [ this ] {this->spectralClusterer[5].similarity.execute();});
thread1.join();
thread2.join();
thread3.join();
thread4.join();
thread5.join();
thread6.join();
spectralClusterer[0].normalizedLaplacian.similarity = spectralClusterer[0].similarity.similarity;
spectralClusterer[0].normalizedLaplacian.degree = spectralClusterer[0].similarity.degree;
spectralClusterer[1].normalizedLaplacian.similarity = spectralClusterer[1].similarity.similarity;
spectralClusterer[1].normalizedLaplacian.degree = spectralClusterer[1].similarity.degree;
spectralClusterer[2].normalizedLaplacian.similarity = spectralClusterer[2].similarity.similarity;
spectralClusterer[2].normalizedLaplacian.degree = spectralClusterer[2].similarity.degree;
spectralClusterer[3].normalizedLaplacian.similarity = spectralClusterer[3].similarity.similarity;
spectralClusterer[3].normalizedLaplacian.degree = spectralClusterer[3].similarity.degree;
spectralClusterer[4].normalizedLaplacian.similarity = spectralClusterer[4].similarity.similarity;
spectralClusterer[4].normalizedLaplacian.degree = spectralClusterer[4].similarity.degree;
spectralClusterer[5].normalizedLaplacian.similarity = spectralClusterer[5].similarity.similarity;
spectralClusterer[5].normalizedLaplacian.degree = spectralClusterer[5].similarity.degree;
std::thread thread7( [ this ] {this->spectralClusterer[0].normalizedLaplacian.execute();});
std::thread thread8( [ this ] {this->spectralClusterer[1].normalizedLaplacian.execute();});
std::thread thread9( [ this ] {this->spectralClusterer[2].normalizedLaplacian.execute();});
std::thread thread10( [ this ] {this->spectralClusterer[3].normalizedLaplacian.execute();});
std::thread thread11( [ this ] {this->spectralClusterer[4].normalizedLaplacian.execute();});
std::thread thread12( [ this ] {this->spectralClusterer[5].normalizedLaplacian.execute();});
thread7.join();
thread8.join();
thread9.join();
thread10.join();
thread11.join();
thread12.join();
spectralClusterer[0].eigenSolver.matrix = spectralClusterer[0].normalizedLaplacian.nLaplacian;
spectralClusterer[1].eigenSolver.matrix = spectralClusterer[1].normalizedLaplacian.nLaplacian;
spectralClusterer[2].eigenSolver.matrix = spectralClusterer[2].normalizedLaplacian.nLaplacian;
spectralClusterer[3].eigenSolver.matrix = spectralClusterer[3].normalizedLaplacian.nLaplacian;
spectralClusterer[4].eigenSolver.matrix = spectralClusterer[4].normalizedLaplacian.nLaplacian;
spectralClusterer[5].eigenSolver.matrix = spectralClusterer[5].normalizedLaplacian.nLaplacian;
std::thread thread13( [ this ] {this->spectralClusterer[0].eigenSolver.execute();});
std::thread thread14( [ this ] {this->spectralClusterer[1].eigenSolver.execute();});
std::thread thread15( [ this ] {this->spectralClusterer[2].eigenSolver.execute();});
std::thread thread16( [ this ] {this->spectralClusterer[3].eigenSolver.execute();});
std::thread thread17( [ this ] {this->spectralClusterer[4].eigenSolver.execute();});
std::thread thread18( [ this ] {this->spectralClusterer[5].eigenSolver.execute();});
thread13.join();
thread14.join();
thread15.join();
thread16.join();
thread17.join();
thread18.join();
spectralClusterer[0].kMeansClustering.vectors = spectralClusterer[0].eigenSolver.eigenvectors;
spectralClusterer[1].kMeansClustering.vectors = spectralClusterer[1].eigenSolver.eigenvectors;
spectralClusterer[2].kMeansClustering.vectors = spectralClusterer[2].eigenSolver.eigenvectors;
spectralClusterer[3].kMeansClustering.vectors = spectralClusterer[3].eigenSolver.eigenvectors;
spectralClusterer[4].kMeansClustering.vectors = spectralClusterer[4].eigenSolver.eigenvectors;
spectralClusterer[5].kMeansClustering.vectors = spectralClusterer[5].eigenSolver.eigenvectors;
std::thread thread19( [ this ] {this->spectralClusterer[0].kMeansClustering.execute();});
std::thread thread20( [ this ] {this->spectralClusterer[1].kMeansClustering.execute();});
std::thread thread21( [ this ] {this->spectralClusterer[2].kMeansClustering.execute();});
std::thread thread22( [ this ] {this->spectralClusterer[3].kMeansClustering.execute();});
std::thread thread23( [ this ] {this->spectralClusterer[4].kMeansClustering.execute();});
std::thread thread24( [ this ] {this->spectralClusterer[5].kMeansClustering.execute();});
thread19.join();
thread20.join();
thread21.join();
thread22.join();
thread23.join();
thread24.join();
clusters[0] = spectralClusterer[0].kMeansClustering.clusters;
clusters[1] = spectralClusterer[1].kMeansClustering.clusters;
clusters[2] = spectralClusterer[2].kMeansClustering.clusters;
clusters[3] = spectralClusterer[3].kMeansClustering.clusters;
clusters[4] = spectralClusterer[4].kMeansClustering.clusters;
clusters[5] = spectralClusterer[5].kMeansClustering.clusters;
}

};
#endif
