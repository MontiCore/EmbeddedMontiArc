#ifndef PAPER_MATHUNIT4
#define PAPER_MATHUNIT4
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "paper_mathUnit4_matrixModifier_1_.h"
#include <thread>
using namespace arma;
class paper_mathUnit4{
public:
mat mat1[4];
mat mat2[4];
mat mat3[4];
mat mat4[4];
mat mat5[4];
mat matOut1[4];
paper_mathUnit4_matrixModifier_1_ matrixModifier[4];
void init()
{
mat1[0]=mat(1000,2);
mat1[1]=mat(1000,2);
mat1[2]=mat(1000,2);
mat1[3]=mat(1000,2);
mat2[0]=mat(2,1000);
mat2[1]=mat(2,1000);
mat2[2]=mat(2,1000);
mat2[3]=mat(2,1000);
mat3[0]=mat(1000,2);
mat3[1]=mat(1000,2);
mat3[2]=mat(1000,2);
mat3[3]=mat(1000,2);
mat4[0]=mat(2,10000);
mat4[1]=mat(2,10000);
mat4[2]=mat(2,10000);
mat4[3]=mat(2,10000);
mat5[0]=mat(10000,10000);
mat5[1]=mat(10000,10000);
mat5[2]=mat(10000,10000);
mat5[3]=mat(10000,10000);
matOut1[0]=mat(1000,10000);
matOut1[1]=mat(1000,10000);
matOut1[2]=mat(1000,10000);
matOut1[3]=mat(1000,10000);
mat1[0]=mat(1000,2);
matrixModifier[0].mat1=mat(1000,2);
mat1[1]=mat(1000,2);
matrixModifier[1].mat1=mat(1000,2);
mat1[2]=mat(1000,2);
matrixModifier[2].mat1=mat(1000,2);
mat1[3]=mat(1000,2);
matrixModifier[3].mat1=mat(1000,2);
mat2[0]=mat(2,1000);
matrixModifier[0].mat2=mat(2,1000);
mat2[1]=mat(2,1000);
matrixModifier[1].mat2=mat(2,1000);
mat2[2]=mat(2,1000);
matrixModifier[2].mat2=mat(2,1000);
mat2[3]=mat(2,1000);
matrixModifier[3].mat2=mat(2,1000);
mat3[0]=mat(1000,2);
matrixModifier[0].mat3=mat(1000,2);
mat3[1]=mat(1000,2);
matrixModifier[1].mat3=mat(1000,2);
mat3[2]=mat(1000,2);
matrixModifier[2].mat3=mat(1000,2);
mat3[3]=mat(1000,2);
matrixModifier[3].mat3=mat(1000,2);
mat4[0]=mat(2,10000);
matrixModifier[0].mat4=mat(2,10000);
mat4[1]=mat(2,10000);
matrixModifier[1].mat4=mat(2,10000);
mat4[2]=mat(2,10000);
matrixModifier[2].mat4=mat(2,10000);
mat4[3]=mat(2,10000);
matrixModifier[3].mat4=mat(2,10000);
mat5[0]=mat(10000,10000);
matrixModifier[0].mat5=mat(10000,10000);
mat5[1]=mat(10000,10000);
matrixModifier[1].mat5=mat(10000,10000);
mat5[2]=mat(10000,10000);
matrixModifier[2].mat5=mat(10000,10000);
mat5[3]=mat(10000,10000);
matrixModifier[3].mat5=mat(10000,10000);
matrixModifier[0].matOut=mat(1000,10000);
matOut1[0]=mat(1000,10000);
matrixModifier[1].matOut=mat(1000,10000);
matOut1[1]=mat(1000,10000);
matrixModifier[2].matOut=mat(1000,10000);
matOut1[2]=mat(1000,10000);
matrixModifier[3].matOut=mat(1000,10000);
matOut1[3]=mat(1000,10000);
matrixModifier[0].init();
matrixModifier[1].init();
matrixModifier[2].init();
matrixModifier[3].init();
}
void execute()
{
matrixModifier[0].mat1 = mat1[0];
matrixModifier[1].mat1 = mat1[1];
matrixModifier[2].mat1 = mat1[2];
matrixModifier[3].mat1 = mat1[3];
matrixModifier[0].mat2 = mat2[0];
matrixModifier[1].mat2 = mat2[1];
matrixModifier[2].mat2 = mat2[2];
matrixModifier[3].mat2 = mat2[3];
matrixModifier[0].mat3 = mat3[0];
matrixModifier[1].mat3 = mat3[1];
matrixModifier[2].mat3 = mat3[2];
matrixModifier[3].mat3 = mat3[3];
matrixModifier[0].mat4 = mat4[0];
matrixModifier[1].mat4 = mat4[1];
matrixModifier[2].mat4 = mat4[2];
matrixModifier[3].mat4 = mat4[3];
matrixModifier[0].mat5 = mat5[0];
matrixModifier[1].mat5 = mat5[1];
matrixModifier[2].mat5 = mat5[2];
matrixModifier[3].mat5 = mat5[3];
std::thread thread1( [ this ] {this->matrixModifier[0].execute();});
std::thread thread2( [ this ] {this->matrixModifier[1].execute();});
std::thread thread3( [ this ] {this->matrixModifier[2].execute();});
std::thread thread4( [ this ] {this->matrixModifier[3].execute();});
thread1.join();
thread2.join();
thread3.join();
thread4.join();
matOut1[0] = matrixModifier[0].matOut;
matOut1[1] = matrixModifier[1].matOut;
matOut1[2] = matrixModifier[2].matOut;
matOut1[3] = matrixModifier[3].matOut;
}

};
#endif
