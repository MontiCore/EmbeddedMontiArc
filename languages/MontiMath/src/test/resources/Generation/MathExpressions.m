// (c) https://github.com/MontiCore/monticore 

package Generation;

script MathExpressions
  Q A= 1+2;
  Q matB= 3 + A;
  Q matC = ((1  + 2 )*(3  + 4 ))^2;
  Q^{2,2} D = ([1,1;1,1]+[2,2;2,2]) * ([3,3;3,3]+[4,4;4,4]);
  Q E = (A+matB)%5;

end
