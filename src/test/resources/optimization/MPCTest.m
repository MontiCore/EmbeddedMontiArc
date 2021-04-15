// (c) https://github.com/MontiCore/monticore

package optimization;

script MPCTest
    minimize<n=1:20>
        Q x;
    in
        Q z = 2 * x(20) + 1;
    subject to
        Q y = 0;

        x(n+1) == x(n) + 1;
        y(n+1) == x(n) * 2;
    end
end
