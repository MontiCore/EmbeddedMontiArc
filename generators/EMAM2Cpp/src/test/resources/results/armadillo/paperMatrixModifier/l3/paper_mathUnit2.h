#ifndef PAPER_MATHUNIT2
#define PAPER_MATHUNIT2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "paper_mathUnit2_matrixModifier_1_.h"
#include <thread>
using namespace arma;
class paper_mathUnit2{
public:
mat mat1[2];
mat mat2[2];
mat mat3[2];
mat mat4[2];
mat mat5[2];
mat matOut1[2];
paper_mathUnit2_matrixModifier_1_ matrixModifier[2];
void init()
{
mat1[0]=mat(1000,200);
mat1[1]=mat(1000,200);
mat2[0]=mat(1000,200);
mat2[1]=mat(1000,200);
mat3[0]=mat(200,10);
mat3[1]=mat(200,10);
mat4[0]=mat(10,100);
mat4[1]=mat(10,100);
mat5[0]=mat(100,50000);
mat5[1]=mat(100,50000);
matOut1[0]=mat(1000,100);
matOut1[1]=mat(1000,100);
mat1[0]=mat(1000,200);
matrixModifier[0].mat1=mat(1000,2);
mat1[1]=mat(1000,200);
matrixModifier[1].mat1=mat(1000,2);
mat2[0]=mat(1000,200);
matrixModifier[0].mat2=mat(2,1000);
mat2[1]=mat(1000,200);
matrixModifier[1].mat2=mat(2,1000);
mat3[0]=mat(200,10);
matrixModifier[0].mat3=mat(1000,2);
mat3[1]=mat(200,10);
matrixModifier[1].mat3=mat(1000,2);
mat4[0]=mat(10,100);
matrixModifier[0].mat4=mat(2,10000);
mat4[1]=mat(10,100);
matrixModifier[1].mat4=mat(2,10000);
mat5[0]=mat(100,50000);
matrixModifier[0].mat5=mat(10000,10000);
mat5[1]=mat(100,50000);
matrixModifier[1].mat5=mat(10000,10000);
matrixModifier[0].matOut=mat(1000,10000);
matOut1[0]=mat(1000,100);
matrixModifier[1].matOut=mat(1000,10000);
matOut1[1]=mat(1000,100);
matrixModifier[0].init();
matrixModifier[1].init();
}
void execute()
{
matrixModifier[0].mat1 = mat1[0];
matrixModifier[1].mat1 = mat1[1];
matrixModifier[0].mat2 = mat2[0];
matrixModifier[1].mat2 = mat2[1];
matrixModifier[0].mat3 = mat3[0];
matrixModifier[1].mat3 = mat3[1];
matrixModifier[0].mat4 = mat4[0];
matrixModifier[1].mat4 = mat4[1];
matrixModifier[0].mat5 = mat5[0];
matrixModifier[1].mat5 = mat5[1];
std::thread thread1( [ this ] {this->matrixModifier[0].execute();});
std::thread thread2( [ this ] {this->matrixModifier[1].execute();});
thread1.join();
thread2.join();
matOut1[0] = matrixModifier[0].matOut;
matOut1[1] = matrixModifier[1].matOut;
}

};
#endif
