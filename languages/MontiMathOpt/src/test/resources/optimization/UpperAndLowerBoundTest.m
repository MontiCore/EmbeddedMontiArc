// (c) https://github.com/MontiCore/monticore 

package optimization;

script UpperAndLowerBoundTest
    minimize
        Q x;
    in
        Q y = x;
    subject to
        0 <= x <= 1;
        x >= -1;
    end
end
