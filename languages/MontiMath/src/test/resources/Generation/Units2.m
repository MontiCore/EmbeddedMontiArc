// (c) https://github.com/MontiCore/monticore 

package Generation;

script Units2
  Q(0 m : 10 m)^{1,2} A = [1 m, 2 m];
  Q(0 m : 10 m)^{2,1} matB = [1 m; 2 m];
  Q(0 m^2 : 100 m^2)^{1,1} matC = A*matB;
  Q(0 m : 10 m) D = 2 m;
  Q(0 m : 10 m) E = 5 m;
  Q(0 m^2 : 200 m^2) F = D*E + 3 m^2;

end
