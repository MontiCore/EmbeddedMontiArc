// (c) https://github.com/MontiCore/monticore 

package Generation;
script Units
  Q(0 m : 10 m) A = 5 m;
  Q(0 m : 10 m) Bmat = 1 m;
  Q(0 m : 10 m)^{2} Cmat = [1 m, 2 m];
  Q(0 m*s : 10 m*s) F = 2 m * 1 s;
  Q(0 m^2 : 10 m^2) D = 1 m*m;
  Q(0 m^4 : 1000 m^4) E = ((1 m + 2 m)*(3 m + 4 m))^2;

end
