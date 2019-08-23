// (c) https://github.com/MontiCore/monticore 

package calculations;

script conditions
     if r == c
        A(r,c) = 2;
     elseif abs(r-c) == 1
        A(r+2,c*3) = -1;
     else
        A(r, c) = 0;
     end
end
