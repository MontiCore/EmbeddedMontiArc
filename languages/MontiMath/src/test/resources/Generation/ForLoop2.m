// (c) https://github.com/MontiCore/monticore 

package Generation;

script ForLoop2
  Q(0 m : 1000 m)^{5} c = [1 m, 3 m , 5 m, 7 m, 9 m];
  Q x = 0;
  Q(0 m^2 : 1000 m^2) y = 0 m*m;
  Q(0 m : 1000 m) z = 0 m;

//TODO fix this to work with variable vectors, and not only vectors that are equivalent to range vectors
  for i = c
   for j = c
     y+=j*i;
     z += c(x+0);
    end
   x+=1;
  end
end
