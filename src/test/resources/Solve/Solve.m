// (c) https://github.com/MontiCore/monticore 

package Solve;

script Solve

Q in1 = 2;

S = solve([diff3=in1-diff2,
        diff2=diff1-gain2,
        gain2=2*diff2,
        diff1=diff3-gain1,
        gain1=3*diff1]);
with
    diff3(0) = 0;
end

Q res = S.diff2;

end
