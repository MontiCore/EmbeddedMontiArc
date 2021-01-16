/* (c) https://github.com/MontiCore/monticore */
#ifndef DETECTION_OBJECTDETECTOR2
#define DETECTION_OBJECTDETECTOR2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "detection_objectDetector2_spectralClusterer_1_.h"
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
spectralClusterer[0].similarity.execute();
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer[1].similarity.execute();
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer[0].normalizedLaplacian.execute();
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer[1].normalizedLaplacian.execute();
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer[0].eigenSolver.execute();
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer[1].eigenSolver.execute();
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer[0].kMeansClustering.execute();
clusters[0] = spectralClusterer.kMeansClustering.clusters;
spectralClusterer[1].kMeansClustering.execute();
clusters[1] = spectralClusterer.kMeansClustering.clusters;
}

};
#endif
