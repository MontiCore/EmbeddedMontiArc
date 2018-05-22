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
const int n = 50;
const int elements = 2500;
const int k = 4;
const int maximumClusters = 1;
public:
mat red;
mat green;
mat blue;
mat clusters;
detection_objectDetector1_spectralClusterer_similarity similarity;
detection_objectDetector1_spectralClusterer_normalizedLaplacian normalizedLaplacian;
detection_objectDetector1_spectralClusterer_eigenSolver eigenSolver;
detection_objectDetector1_spectralClusterer_kMeansClustering kMeansClustering;
void init()
{
red=mat(n,n);
green=mat(n,n);
blue=mat(n,n);
clusters=mat(elements,maximumClusters);
similarity.init();
normalizedLaplacian.init();
eigenSolver.init();
kMeansClustering.init();
}
void execute()
{
similarity.red = red;
similarity.green = green;
similarity.blue = blue;
similarity.execute();
normalizedLaplacian.degree = similarity.degree;
normalizedLaplacian.similarity = similarity.similarity;
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
