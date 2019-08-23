// (c) https://github.com/MontiCore/monticore 

package optimization;

script ForLoopConditionTest
    Q y = minimize(Q^{3} x)
        x(1)^2 + x(2)^2 + x(3)^2;
    subject to
        for i = 1:3
            x(i) >= 0;
            0 <= x(i) <= 1;
        end
    end
end
