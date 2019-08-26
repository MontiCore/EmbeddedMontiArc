// (c) https://github.com/MontiCore/monticore 

package calculations;

script example2

    Q(0:10)^{1,5} c = 1:2:10;
    Q x = 0;
    Q y = 0;

    for i = c
      for j = c
        y+=c(x)*j+i;
      end
      x+=1;
    end
end
