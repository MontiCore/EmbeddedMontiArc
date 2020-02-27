#ifndef TEST_MATH_LARGESTCONTOURCOMMANDTEST
#define TEST_MATH_LARGESTCONTOURCOMMANDTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "opencv2/imgproc.hpp"
using namespace arma;
using namespace std;
class test_math_largestContourCommandTest{
public:
std::vector<std::vector<cv::Point>> contours;
std::vector<cv::Point> lContour;
void init()
{
}
std::vector<cv::Point> largestContour(const std::vector<std::vector<cv::Point>>& contours)
{
double maxArea = 0;
int maxAreaContourId = -1;
   for (int j = 0; j < contours.size(); j++) {
       double newArea = cv::contourArea(contours.at(j));
       if (newArea > maxArea) {
           maxArea = newArea;
           maxAreaContourId = j;
       }
   }
   return contours.at(maxAreaContourId);
}
void execute()
{
lContour = (largestContour(contours));
}

};
#endif
