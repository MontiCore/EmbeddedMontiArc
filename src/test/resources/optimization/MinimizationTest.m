// (c) https://github.com/MontiCore/monticore 

package optimization;

script MinimizationTest
    minimize(Q x)
        x^2;
    subject to
        x <= 1;
    end
end
