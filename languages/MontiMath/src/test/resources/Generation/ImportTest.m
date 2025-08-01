// (c) https://github.com/MontiCore/monticore 

package Generation;

import Calculations.*;

script ImportTest
  Q  A =1;
  Q^{2,2} matB = [0,0;0,0];
  A = 2;
  matB = [1,2;3,4];
  Q^{2,2} matC = [1,1;1,1];
  matC = matB;
  Q D = A;

end
