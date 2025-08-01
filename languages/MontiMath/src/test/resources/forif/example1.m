// (c) https://github.com/MontiCore/monticore 

package forif;

script example1
    Q nrows = 4;
    Q ncols = 6;
    Q^{nrows, ncols} A;
    for c = 1:ncols
        for r = 1:nrows
            if r == c
                A(r,c) = 2;
            elseif abs(r-c) == 1
                A(r+2,c*3) = -1;
            else
              A(r, c) = 0;
            end
        end
    end
end
