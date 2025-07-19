#ifndef CNNSEGMENT_CONNECTOR_DEONEHOT
#define CNNSEGMENT_CONNECTOR_DEONEHOT
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
using namespace std;
class cNNSegment_connector_deonehot{
const int channels = 10;
const int rows = 28;
const int cols = 28;
public:
cube inputVector;
cube maxValues;
icube res;
void init()
{
inputVector = cube(channels, rows, cols);
maxValues = cube(1, rows, cols);
res = icube(1, rows, cols);
}
void execute()
{
for( auto row=1;row<=rows;++row){
for( auto col=1;col<=cols;++col){
std::cout << "deonehot current index: (" << 0 << "," << row-1 << "," << col-1 << ")\n";
std::cout << "deonehot maxValues: " << maxValues(1-1, row-1, col-1)<< "\n";
std::cout << "deonehot inputVector: " << inputVector(1-1, row-1, col-1)<< "\n";
std::cout << "deonehot res: " << res(1-1, row-1, col-1)<< "\n";
maxValues(1-1, row-1, col-1) = inputVector(1-1, row-1, col-1);
res(1-1, row-1, col-1) = 0;
for( auto i=2;i<=channels;++i){
std::cout << "deonehot current index: (" << i-1 << "," << row-1 << "," << col-1 << ")\n";
std::cout << "deonehot current index: (" << row-1 << "," << col-1 << "," << i-1 << ")\n";
std::cout << "deonehot inputVector: " << inputVector(row-1, col-1, i-1)<< "\n";
std::cout << "deonehot maxValues: " << maxValues(1-1, row-1, col-1)<< "\n";
std::cout << "deonehot res: " << res(1-1, row-1, col-1)<< "\n";
if((inputVector(row-1, col-1, i-1) > maxValues(i-1, col-1, row-1))){
maxValues(1-1, row-1, col-1) = inputVector(row-1, col-1, i-1);
res(1-1, row-1, col-1) = i;
}
}
}
}
}

};
#endif
