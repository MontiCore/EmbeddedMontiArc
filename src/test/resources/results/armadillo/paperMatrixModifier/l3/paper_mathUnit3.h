#ifndef PAPER_MATHUNIT3
#define PAPER_MATHUNIT3
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "paper_mathUnit3_matrixModifier_1_.h"
#include <thread>
using namespace arma;
class paper_mathUnit3{
public:
mat mat1[3];
mat mat2[3];
mat mat3[3];
mat mat4[3];
double factor[3];
mat matOut1[3];
paper_mathUnit3_matrixModifier_1_ matrixModifier[3];
void init()
{
mat1[0]=mat(1000,200);
mat1[1]=mat(1000,200);
mat1[2]=mat(1000,200);
mat2[0]=mat(1000,200);
mat2[1]=mat(1000,200);
mat2[2]=mat(1000,200);
mat3[0]=mat(200,10);
mat3[1]=mat(200,10);
mat3[2]=mat(200,10);
mat4[0]=mat(10,100);
mat4[1]=mat(10,100);
mat4[2]=mat(10,100);
matOut1[0]=mat(1000,100);
matOut1[1]=mat(1000,100);
matOut1[2]=mat(1000,100);
matrixModifier[0].init();
matrixModifier[1].init();
matrixModifier[2].init();
}
void execute()
{
matrixModifier[0].mat1 = mat1[0];
matrixModifier[0].mat2 = mat2[0];
matrixModifier[0].mat3 = mat3[0];
matrixModifier[0].mat4 = mat4[0];
matrixModifier[0].factor = factor[0];
std::thread thread1( [ this ] {this->matrixModifier[0].execute();});
matrixModifier[1].mat1 = mat1[1];
matrixModifier[1].mat2 = mat2[1];
matrixModifier[1].mat3 = mat3[1];
matrixModifier[1].mat4 = mat4[1];
matrixModifier[1].factor = factor[1];
std::thread thread2( [ this ] {this->matrixModifier[1].execute();});
matrixModifier[2].mat1 = mat1[2];
matrixModifier[2].mat2 = mat2[2];
matrixModifier[2].mat3 = mat3[2];
matrixModifier[2].mat4 = mat4[2];
matrixModifier[2].factor = factor[2];
std::thread thread3( [ this ] {this->matrixModifier[2].execute();});
thread1.join();
thread2.join();
thread3.join();
matOut1[0] = matrixModifier[0].matOut;
matOut1[1] = matrixModifier[1].matOut;
matOut1[2] = matrixModifier[2].matOut;
}

};
#endif
