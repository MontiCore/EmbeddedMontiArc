// (c) https://github.com/MontiCore/monticore 

package Generation;

script ForLoop3
  Q^{3,3} A = [1,2,3;4,5,6; 7,8,9];

  Q Cmat = 0;

  for i = 0:2
    for j = 0:2
      Cmat += A(i,j);
    end
  end
end
