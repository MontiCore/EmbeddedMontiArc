#ifndef PAPER_MATHUNIT1
#define PAPER_MATHUNIT1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "paper_mathUnit1_matrixModifier.h"
#include <thread>
using namespace arma;
class paper_mathUnit1{
public:
mat mat1;
mat mat2;
mat mat3;
mat mat4;
mat mat5;
mat matOut1;
paper_mathUnit1_matrixModifier matrixModifier;
void init()
{
mat1=mat(1000,200);
mat2=mat(1000,200);
mat3=mat(200,10);
mat4=mat(10,100);
mat5=mat(100,50000);
matOut1=mat(1000,100);
mat1=mat(1000,200);
matrixModifier.mat1=mat(1000,2);
mat2=mat(1000,200);
matrixModifier.mat2=mat(2,1000);
mat3=mat(200,10);
matrixModifier.mat3=mat(1000,2);
mat4=mat(10,100);
matrixModifier.mat4=mat(2,10000);
mat5=mat(100,50000);
matrixModifier.mat5=mat(10000,10000);
matrixModifier.matOut=mat(1000,10000);
matOut1=mat(1000,100);
matrixModifier.init();
}
void execute()
{
matrixModifier.mat1 = mat1;
matrixModifier.mat2 = mat2;
matrixModifier.mat3 = mat3;
matrixModifier.mat4 = mat4;
matrixModifier.mat5 = mat5;
std::thread thread1( [ this ] {this->matrixModifier.execute();});
thread1.join();
matOut1 = matrixModifier.matOut;
}

};
#endif
