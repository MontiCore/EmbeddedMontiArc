// (c) https://github.com/MontiCore/monticore 

package matrix;

script CreateMatrixBySubmatrices
  Q^{2,2} A = [1, 2; 3, 4];
  Q^{2,2} B = [A(1,1), A(1,2), A(2,1), A(2,2)];
end
