/* (c) https://github.com/MontiCore/monticore */
#ifndef DETECTION_OBJECTDETECTOR8
#define DETECTION_OBJECTDETECTOR8
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "detection_objectDetector8_spectralClusterer_1_.h"
using namespace arma;
class detection_objectDetector8{
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
mat clusters[8];
detection_objectDetector8_spectralClusterer_1_ spectralClusterer[8];
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
clusters[0]=mat(2500,1);
clusters[1]=mat(2500,1);
clusters[2]=mat(2500,1);
clusters[3]=mat(2500,1);
clusters[4]=mat(2500,1);
clusters[5]=mat(2500,1);
clusters[6]=mat(2500,1);
clusters[7]=mat(2500,1);
spectralClusterer[0].init();
spectralClusterer[1].init();
spectralClusterer[2].init();
spectralClusterer[3].init();
spectralClusterer[4].init();
spectralClusterer[5].init();
spectralClusterer[6].init();
spectralClusterer[7].init();
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
spectralClusterer[6].similarity.red = red7;
spectralClusterer[6].similarity.green = green7;
spectralClusterer[6].similarity.blue = blue7;
spectralClusterer[7].similarity.red = red8;
spectralClusterer[7].similarity.green = green8;
spectralClusterer[7].similarity.blue = blue8;
spectralClusterer[0].similarity.execute();
spectralClusterer[0].normalizedLaplacian.similarity = spectralClusterer[0].similarity.similarity;
spectralClusterer[0].normalizedLaplacian.degree = spectralClusterer[0].similarity.degree;
spectralClusterer[1].similarity.execute();
spectralClusterer[1].normalizedLaplacian.similarity = spectralClusterer[1].similarity.similarity;
spectralClusterer[1].normalizedLaplacian.degree = spectralClusterer[1].similarity.degree;
spectralClusterer[2].similarity.execute();
spectralClusterer[2].normalizedLaplacian.similarity = spectralClusterer[2].similarity.similarity;
spectralClusterer[2].normalizedLaplacian.degree = spectralClusterer[2].similarity.degree;
spectralClusterer[3].similarity.execute();
spectralClusterer[3].normalizedLaplacian.similarity = spectralClusterer[3].similarity.similarity;
spectralClusterer[3].normalizedLaplacian.degree = spectralClusterer[3].similarity.degree;
spectralClusterer[4].similarity.execute();
spectralClusterer[4].normalizedLaplacian.similarity = spectralClusterer[4].similarity.similarity;
spectralClusterer[4].normalizedLaplacian.degree = spectralClusterer[4].similarity.degree;
spectralClusterer[5].similarity.execute();
spectralClusterer[5].normalizedLaplacian.similarity = spectralClusterer[5].similarity.similarity;
spectralClusterer[5].normalizedLaplacian.degree = spectralClusterer[5].similarity.degree;
spectralClusterer[6].similarity.execute();
spectralClusterer[6].normalizedLaplacian.similarity = spectralClusterer[6].similarity.similarity;
spectralClusterer[6].normalizedLaplacian.degree = spectralClusterer[6].similarity.degree;
spectralClusterer[7].similarity.execute();
spectralClusterer[7].normalizedLaplacian.similarity = spectralClusterer[7].similarity.similarity;
spectralClusterer[7].normalizedLaplacian.degree = spectralClusterer[7].similarity.degree;
spectralClusterer[0].normalizedLaplacian.execute();
spectralClusterer[0].eigenSolver.matrix = spectralClusterer[0].normalizedLaplacian.nLaplacian;
spectralClusterer[1].normalizedLaplacian.execute();
spectralClusterer[1].eigenSolver.matrix = spectralClusterer[1].normalizedLaplacian.nLaplacian;
spectralClusterer[2].normalizedLaplacian.execute();
spectralClusterer[2].eigenSolver.matrix = spectralClusterer[2].normalizedLaplacian.nLaplacian;
spectralClusterer[3].normalizedLaplacian.execute();
spectralClusterer[3].eigenSolver.matrix = spectralClusterer[3].normalizedLaplacian.nLaplacian;
spectralClusterer[4].normalizedLaplacian.execute();
spectralClusterer[4].eigenSolver.matrix = spectralClusterer[4].normalizedLaplacian.nLaplacian;
spectralClusterer[5].normalizedLaplacian.execute();
spectralClusterer[5].eigenSolver.matrix = spectralClusterer[5].normalizedLaplacian.nLaplacian;
spectralClusterer[6].normalizedLaplacian.execute();
spectralClusterer[6].eigenSolver.matrix = spectralClusterer[6].normalizedLaplacian.nLaplacian;
spectralClusterer[7].normalizedLaplacian.execute();
spectralClusterer[7].eigenSolver.matrix = spectralClusterer[7].normalizedLaplacian.nLaplacian;
spectralClusterer[0].eigenSolver.execute();
spectralClusterer[0].kMeansClustering.vectors = spectralClusterer[0].eigenSolver.eigenvectors;
spectralClusterer[1].eigenSolver.execute();
spectralClusterer[1].kMeansClustering.vectors = spectralClusterer[1].eigenSolver.eigenvectors;
spectralClusterer[2].eigenSolver.execute();
spectralClusterer[2].kMeansClustering.vectors = spectralClusterer[2].eigenSolver.eigenvectors;
spectralClusterer[3].eigenSolver.execute();
spectralClusterer[3].kMeansClustering.vectors = spectralClusterer[3].eigenSolver.eigenvectors;
spectralClusterer[4].eigenSolver.execute();
spectralClusterer[4].kMeansClustering.vectors = spectralClusterer[4].eigenSolver.eigenvectors;
spectralClusterer[5].eigenSolver.execute();
spectralClusterer[5].kMeansClustering.vectors = spectralClusterer[5].eigenSolver.eigenvectors;
spectralClusterer[6].eigenSolver.execute();
spectralClusterer[6].kMeansClustering.vectors = spectralClusterer[6].eigenSolver.eigenvectors;
spectralClusterer[7].eigenSolver.execute();
spectralClusterer[7].kMeansClustering.vectors = spectralClusterer[7].eigenSolver.eigenvectors;
spectralClusterer[0].kMeansClustering.execute();
clusters[0] = spectralClusterer[0].kMeansClustering.clusters;
spectralClusterer[1].kMeansClustering.execute();
clusters[1] = spectralClusterer[1].kMeansClustering.clusters;
spectralClusterer[2].kMeansClustering.execute();
clusters[2] = spectralClusterer[2].kMeansClustering.clusters;
spectralClusterer[3].kMeansClustering.execute();
clusters[3] = spectralClusterer[3].kMeansClustering.clusters;
spectralClusterer[4].kMeansClustering.execute();
clusters[4] = spectralClusterer[4].kMeansClustering.clusters;
spectralClusterer[5].kMeansClustering.execute();
clusters[5] = spectralClusterer[5].kMeansClustering.clusters;
spectralClusterer[6].kMeansClustering.execute();
clusters[6] = spectralClusterer[6].kMeansClustering.clusters;
spectralClusterer[7].kMeansClustering.execute();
clusters[7] = spectralClusterer[7].kMeansClustering.clusters;
}

};
#endif
