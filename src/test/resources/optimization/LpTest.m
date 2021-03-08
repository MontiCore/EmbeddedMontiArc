// (c) https://github.com/MontiCore/monticore 

package optimization;

// transportation problem example (linear)
script LpTest
    // define problem
    Q m = 3;
    Q n = 2;

    // define A, b
    Q^{3, 1} A = [45; 60; 35];
    Q^{2, 1} b = [50; 60];

    // cost matrix
    Q ^{m, n} c = [3, 2; 1, 5; 5, 4];

    // minimization problem
    minimize
        Q^{3, 2} x;
    in
        sum(c .* x);
    subject to
        sum(x, 2) == A;
        sum(x, 1) == b;
        x >= 0;
    end
end
