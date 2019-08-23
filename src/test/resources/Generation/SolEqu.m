// (c) https://github.com/MontiCore/monticore 

package Generation;
script SolEqu

  Q^{3,1} A = [3 6 2; 1 2 8; 7 9 4] \ [2;3;4];
  Q^{3,1} Bmat = ([3/2 6/2 2/2; 1/2 2/2 8/2; 7/2 9/2 4/2]+[3/2 6/2 2/2; 1/2 2/2 8/2; 7/2 9/2 4/2]) \ [2;3;4];
  //Q(0 m : 100 m)^{3,1} C = [3 m 6 m 2 m; 1 m 2 m 8 m; 7 m 9 m 4 m] \ [2 m*m;3 m*m;4 m*m];

end
