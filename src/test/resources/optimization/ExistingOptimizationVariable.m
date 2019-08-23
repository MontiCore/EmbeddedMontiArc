// (c) https://github.com/MontiCore/monticore 

package optimization;

script ExistingOptimizationVariable
    // 1. scalar
    Q x = 3;
    Q y = minimize(x)
        2 * x + 1;
    subject to
        -1 <= x <= 1;
    end
    // 2. matrix
    Q^{3,3} a = zeros(3,3);
    Q b = minimize(a)
        a * a';
    subject to
        -10 <= x <= 10;
    end
    // 3. substituted
    Q squared = a * a;
    Q b = minimize(a)
        squared * squared';
    subject to
        -10 <= x <= 10;
    end
end
