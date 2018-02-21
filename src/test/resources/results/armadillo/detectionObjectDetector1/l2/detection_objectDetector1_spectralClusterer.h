#ifndef DETECTION_OBJECTDETECTOR1_SPECTRALCLUSTERER
#define DETECTION_OBJECTDETECTOR1_SPECTRALCLUSTERER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "detection_objectDetector1_spectralClusterer_similarity.h"
#include "detection_objectDetector1_spectralClusterer_normalizedLaplacian.h"
#include "detection_objectDetector1_spectralClusterer_eigenSolver.h"
#include "detection_objectDetector1_spectralClusterer_kMeansClustering.h"
#include <thread>
using namespace arma;
class detection_objectDetector1_spectralClusterer{
const int n = 2500;
const int k = 4;
const int maximumClusters = 4;
public:
mat imgMatrix;
mat clusters;
detection_objectDetector1_spectralClusterer_similarity similarity;
detection_objectDetector1_spectralClusterer_normalizedLaplacian normalizedLaplacian;
detection_objectDetector1_spectralClusterer_eigenSolver eigenSolver;
detection_objectDetector1_spectralClusterer_kMeansClustering kMeansClustering;
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
std::thread thread2( [ this ] {this->kMeansClustering.execute();});
thread2.join();
clusters = kMeansClustering.clusters;
}

};
#endif
