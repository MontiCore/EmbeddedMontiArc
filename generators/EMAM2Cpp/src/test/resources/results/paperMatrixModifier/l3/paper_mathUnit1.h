/* (c) https://github.com/MontiCore/monticore */
#ifndef PAPER_MATHUNIT1
#define PAPER_MATHUNIT1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "paper_mathUnit1_matrixModifier.h"
#include <thread>
class paper_mathUnit1{
public:
Matrix mat1;
Matrix mat2;
Matrix mat3;
Matrix mat4;
Matrix mat5;
Matrix matOut1;
paper_mathUnit1_matrixModifier matrixModifier;
void init()
{
mat1=Matrix(1000,200);
mat2=Matrix(1000,200);
mat3=Matrix(200,10);
mat4=Matrix(10,100);
mat5=Matrix(100,50000);
matOut1=Matrix(1000,100);
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
