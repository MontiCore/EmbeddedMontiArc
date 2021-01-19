#ifndef DETECTION_OBJECTDETECTOR5
#define DETECTION_OBJECTDETECTOR5
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
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
spectralClusterer.similarity.red = red1;
spectralClusterer.similarity.green = green1;
spectralClusterer.similarity.blue = blue1;
spectralClusterer.similarity.red = red2;
spectralClusterer.similarity.green = green2;
spectralClusterer.similarity.blue = blue2;
spectralClusterer.similarity.red = red3;
spectralClusterer.similarity.green = green3;
spectralClusterer.similarity.blue = blue3;
spectralClusterer.similarity.red = red4;
spectralClusterer.similarity.green = green4;
spectralClusterer.similarity.blue = blue4;
spectralClusterer.similarity.red = red5;
spectralClusterer.similarity.green = green5;
spectralClusterer.similarity.blue = blue5;
std::thread thread1( [ this ] {this->spectralClusterer[0].similarity.execute();});
std::thread thread2( [ this ] {this->spectralClusterer[1].similarity.execute();});
std::thread thread3( [ this ] {this->spectralClusterer[2].similarity.execute();});
std::thread thread4( [ this ] {this->spectralClusterer[3].similarity.execute();});
std::thread thread5( [ this ] {this->spectralClusterer[4].similarity.execute();});
thread1.join();
thread2.join();
thread3.join();
thread4.join();
thread5.join();
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
std::thread thread6( [ this ] {this->spectralClusterer[0].normalizedLaplacian.execute();});
std::thread thread7( [ this ] {this->spectralClusterer[1].normalizedLaplacian.execute();});
std::thread thread8( [ this ] {this->spectralClusterer[2].normalizedLaplacian.execute();});
std::thread thread9( [ this ] {this->spectralClusterer[3].normalizedLaplacian.execute();});
std::thread thread10( [ this ] {this->spectralClusterer[4].normalizedLaplacian.execute();});
thread6.join();
thread7.join();
thread8.join();
thread9.join();
thread10.join();
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
std::thread thread11( [ this ] {this->spectralClusterer[0].eigenSolver.execute();});
std::thread thread12( [ this ] {this->spectralClusterer[1].eigenSolver.execute();});
std::thread thread13( [ this ] {this->spectralClusterer[2].eigenSolver.execute();});
std::thread thread14( [ this ] {this->spectralClusterer[3].eigenSolver.execute();});
std::thread thread15( [ this ] {this->spectralClusterer[4].eigenSolver.execute();});
thread11.join();
thread12.join();
thread13.join();
thread14.join();
thread15.join();
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
std::thread thread16( [ this ] {this->spectralClusterer[0].kMeansClustering.execute();});
std::thread thread17( [ this ] {this->spectralClusterer[1].kMeansClustering.execute();});
std::thread thread18( [ this ] {this->spectralClusterer[2].kMeansClustering.execute();});
std::thread thread19( [ this ] {this->spectralClusterer[3].kMeansClustering.execute();});
std::thread thread20( [ this ] {this->spectralClusterer[4].kMeansClustering.execute();});
thread16.join();
thread17.join();
thread18.join();
thread19.join();
thread20.join();
clusters[0] = spectralClusterer.kMeansClustering.clusters;
clusters[1] = spectralClusterer.kMeansClustering.clusters;
clusters[2] = spectralClusterer.kMeansClustering.clusters;
clusters[3] = spectralClusterer.kMeansClustering.clusters;
clusters[4] = spectralClusterer.kMeansClustering.clusters;
}

};
#endif
