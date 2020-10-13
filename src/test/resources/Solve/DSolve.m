// (c) https://github.com/MontiCore/monticore 

package Solve;

script DSolve

S = dsolve([diff(y)=yDiff,
        diff(yDiff)=yDiffDiff,
        yDiffDiff=-2/5*y]);
with
    y(0) = 2;
    yDiff(0) = 0;
end

Q res = S.y;

end
