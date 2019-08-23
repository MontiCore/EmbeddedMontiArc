// (c) https://github.com/MontiCore/monticore 

package forif;

script example2

    Q^{1,5} c = 1:2:10;
    Q x = 0;
    for i = c
      x += i * i;
    end
end
