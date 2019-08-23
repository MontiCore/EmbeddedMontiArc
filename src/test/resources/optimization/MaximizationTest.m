// (c) https://github.com/MontiCore/monticore 

package optimization;

script MaximizationTest
    Q y = maximize(Q x)
        x^2;
    subject to
        x <= 1;
    end
end
