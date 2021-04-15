// (c) https://github.com/MontiCore/monticore 

package optimization;

script MaximizationTest
    maximize
        Q x;
    in
        Q y = x^2;
    subject to
        x <= 1;
    end
end
