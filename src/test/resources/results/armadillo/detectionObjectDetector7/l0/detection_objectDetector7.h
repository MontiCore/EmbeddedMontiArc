#ifndef DETECTION_OBJECTDETECTOR7
#define DETECTION_OBJECTDETECTOR7
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "detection_objectDetector7_spectralClusterer_1_.h"
using namespace arma;
class detection_objectDetector7{
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
mat clusters[7];
detection_objectDetector7_spectralClusterer_1_ spectralClusterer[7];
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
spectralClusterer.similarity.red = red6;
spectralClusterer.similarity.green = green6;
spectralClusterer.similarity.blue = blue6;
spectralClusterer.similarity.red = red7;
spectralClusterer.similarity.green = green7;
spectralClusterer.similarity.blue = blue7;
spectralClusterer[0].similarity.execute();
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer[1].similarity.execute();
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer[2].similarity.execute();
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer[3].similarity.execute();
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer[4].similarity.execute();
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer[5].similarity.execute();
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer[6].similarity.execute();
spectralClusterer.normalizedLaplacian.similarity = spectralClusterer.similarity.similarity;
spectralClusterer.normalizedLaplacian.degree = spectralClusterer.similarity.degree;
spectralClusterer[0].normalizedLaplacian.execute();
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer[1].normalizedLaplacian.execute();
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer[2].normalizedLaplacian.execute();
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer[3].normalizedLaplacian.execute();
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer[4].normalizedLaplacian.execute();
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer[5].normalizedLaplacian.execute();
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer[6].normalizedLaplacian.execute();
spectralClusterer.eigenSolver.matrix = spectralClusterer.normalizedLaplacian.nLaplacian;
spectralClusterer[0].eigenSolver.execute();
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer[1].eigenSolver.execute();
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer[2].eigenSolver.execute();
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer[3].eigenSolver.execute();
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer[4].eigenSolver.execute();
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer[5].eigenSolver.execute();
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer[6].eigenSolver.execute();
spectralClusterer.kMeansClustering.vectors = spectralClusterer.eigenSolver.eigenvectors;
spectralClusterer[0].kMeansClustering.execute();
clusters[0] = spectralClusterer.kMeansClustering.clusters;
spectralClusterer[1].kMeansClustering.execute();
clusters[1] = spectralClusterer.kMeansClustering.clusters;
spectralClusterer[2].kMeansClustering.execute();
clusters[2] = spectralClusterer.kMeansClustering.clusters;
spectralClusterer[3].kMeansClustering.execute();
clusters[3] = spectralClusterer.kMeansClustering.clusters;
spectralClusterer[4].kMeansClustering.execute();
clusters[4] = spectralClusterer.kMeansClustering.clusters;
spectralClusterer[5].kMeansClustering.execute();
clusters[5] = spectralClusterer.kMeansClustering.clusters;
spectralClusterer[6].kMeansClustering.execute();
clusters[6] = spectralClusterer.kMeansClustering.clusters;
}

};
#endif
