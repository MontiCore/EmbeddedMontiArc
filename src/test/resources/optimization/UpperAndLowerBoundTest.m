// (c) https://github.com/MontiCore/monticore 

package optimization;

script UpperAndLowerBoundTest
    Q y = minimize(Q x)
        x;
    subject to
        0 <= x <= 1;
        x >= -1;
    end
end
