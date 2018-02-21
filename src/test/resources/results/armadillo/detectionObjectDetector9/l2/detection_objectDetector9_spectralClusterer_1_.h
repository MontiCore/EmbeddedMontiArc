#ifndef DETECTION_OBJECTDETECTOR9_SPECTRALCLUSTERER_1_
#define DETECTION_OBJECTDETECTOR9_SPECTRALCLUSTERER_1_
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "detection_objectDetector9_spectralClusterer_1__similarity.h"
#include "detection_objectDetector9_spectralClusterer_1__normalizedLaplacian.h"
#include "detection_objectDetector9_spectralClusterer_1__eigenSolver.h"
#include "detection_objectDetector9_spectralClusterer_1__kMeansClustering.h"
#include <thread>
using namespace arma;
class detection_objectDetector9_spectralClusterer_1_{
const int n = 2500;
const int k = 4;
const int maximumClusters = 4;
public:
mat imgMatrix;
mat clusters;
detection_objectDetector9_spectralClusterer_1__similarity similarity;
detection_objectDetector9_spectralClusterer_1__normalizedLaplacian normalizedLaplacian;
detection_objectDetector9_spectralClusterer_1__eigenSolver eigenSolver;
detection_objectDetector9_spectralClusterer_1__kMeansClustering kMeansClustering;
void init()
{
imgMatrix=mat(n*n,3);
clusters=mat(n,maximumClusters);
similarity.init();
normalizedLaplacian.init();
eigenSolver.init();
kMeansClustering.init();
}
void execute()
{
similarity.data = imgMatrix;
similarity.execute();
normalizedLaplacian.degree = similarity.degree;
normalizedLaplacian.W = similarity.similarity;
normalizedLaplacian.execute();
eigenSolver.matrix = normalizedLaplacian.nLaplacian;
eigenSolver.execute();
kMeansClustering.vectors = eigenSolver.eigenvectors;
std::thread thread10( [ this ] {this->kMeansClustering.execute();});
thread10.join();
clusters = kMeansClustering.clusters;
}

};
#endif
