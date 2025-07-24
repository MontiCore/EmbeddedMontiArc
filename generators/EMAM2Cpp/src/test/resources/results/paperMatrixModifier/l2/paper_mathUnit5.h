/* (c) https://github.com/MontiCore/monticore */
#ifndef PAPER_MATHUNIT5
#define PAPER_MATHUNIT5
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "paper_mathUnit5_matrixModifier_1_.h"
#include <thread>
using namespace arma;
class paper_mathUnit5{
public:
mat mat1[5];
mat mat2[5];
mat mat3[5];
mat mat4[5];
mat mat5[5];
mat matOut1[5];
paper_mathUnit5_matrixModifier_1_ matrixModifier[5];
void init()
{
mat1[0]=mat(1000,200);
mat1[1]=mat(1000,200);
mat1[2]=mat(1000,200);
mat1[3]=mat(1000,200);
mat1[4]=mat(1000,200);
mat2[0]=mat(1000,200);
mat2[1]=mat(1000,200);
mat2[2]=mat(1000,200);
mat2[3]=mat(1000,200);
mat2[4]=mat(1000,200);
mat3[0]=mat(200,10);
mat3[1]=mat(200,10);
mat3[2]=mat(200,10);
mat3[3]=mat(200,10);
mat3[4]=mat(200,10);
mat4[0]=mat(10,100);
mat4[1]=mat(10,100);
mat4[2]=mat(10,100);
mat4[3]=mat(10,100);
mat4[4]=mat(10,100);
mat5[0]=mat(100,50000);
mat5[1]=mat(100,50000);
mat5[2]=mat(100,50000);
mat5[3]=mat(100,50000);
mat5[4]=mat(100,50000);
matOut1[0]=mat(1000,100);
matOut1[1]=mat(1000,100);
matOut1[2]=mat(1000,100);
matOut1[3]=mat(1000,100);
matOut1[4]=mat(1000,100);
matrixModifier[0].init();
matrixModifier[1].init();
matrixModifier[2].init();
matrixModifier[3].init();
matrixModifier[4].init();
}
void execute()
{
matrixModifier[0].mat1 = mat1[0];
matrixModifier[1].mat1 = mat1[1];
matrixModifier[2].mat1 = mat1[2];
matrixModifier[3].mat1 = mat1[3];
matrixModifier[4].mat1 = mat1[4];
matrixModifier[0].mat2 = mat2[0];
matrixModifier[1].mat2 = mat2[1];
matrixModifier[2].mat2 = mat2[2];
matrixModifier[3].mat2 = mat2[3];
matrixModifier[4].mat2 = mat2[4];
matrixModifier[0].mat3 = mat3[0];
matrixModifier[1].mat3 = mat3[1];
matrixModifier[2].mat3 = mat3[2];
matrixModifier[3].mat3 = mat3[3];
matrixModifier[4].mat3 = mat3[4];
matrixModifier[0].mat4 = mat4[0];
matrixModifier[1].mat4 = mat4[1];
matrixModifier[2].mat4 = mat4[2];
matrixModifier[3].mat4 = mat4[3];
matrixModifier[4].mat4 = mat4[4];
matrixModifier[0].mat5 = mat5[0];
matrixModifier[1].mat5 = mat5[1];
matrixModifier[2].mat5 = mat5[2];
matrixModifier[3].mat5 = mat5[3];
matrixModifier[4].mat5 = mat5[4];
std::thread thread1( [ this ] {this->matrixModifier[0].execute();});
std::thread thread2( [ this ] {this->matrixModifier[1].execute();});
std::thread thread3( [ this ] {this->matrixModifier[2].execute();});
std::thread thread4( [ this ] {this->matrixModifier[3].execute();});
std::thread thread5( [ this ] {this->matrixModifier[4].execute();});
thread1.join();
thread2.join();
thread3.join();
thread4.join();
thread5.join();
matOut1[0] = matrixModifier[0].matOut;
matOut1[1] = matrixModifier[1].matOut;
matOut1[2] = matrixModifier[2].matOut;
matOut1[3] = matrixModifier[3].matOut;
matOut1[4] = matrixModifier[4].matOut;
}

};
#endif
