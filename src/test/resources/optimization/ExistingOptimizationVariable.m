// (c) https://github.com/MontiCore/monticore 

package optimization;

script ExistingOptimizationVariable
// 1. scalar
        minimize
            Q x = 3;
        in
            2 * x + 1;
        subject to
            -1 <= x <= 1;
        end
        xOut1 = x;

        // 2. matrix
        minimize
            Q^{3,3} a = zeros(3,3);
        in
            Q b = a(1, 1) * a(2, 2) * a(3, 3);
        subject to
            -10 <= a <= 10;
        end
        aOut2 = a;
        bOut2 = b;

        // 3. substituted

        Q c = minimize
            Q^{3,3} a = zeros(3,3);
        in
            ainc(1, 1) * ainc(2, 2) * ainc(3, 3);
        subject to
            Q^{3,3} ainc = a + 1;
            -20 <= a <= 20;
        end
end
