// (c) https://github.com/MontiCore/monticore 

package Generation;

script MatrixWithMathExpression
  Q matB = 5;
  Q matC = 3;
  Q^{2,2} A = [1+1,1*2;1-matC,1/matB];
  A += A^2;
  Q^{3,2} D = ([1,2,3;4,5,6]+[1,2,3;4,5,6])';
end
