// (c) https://github.com/MontiCore/monticore 

package matrix;

script MatrixPropertiesPos

//right declarations
herm square Q^{3,3} a1 = [2,1,0;1,2,0;0,0,2];
diag square Q^{3,3} a2 = [2+1,0,0;0,2/3,0;0,0,2*2];
skewHerm Q^{3,3} a3 = [0,1,0;-1,0,-3;0,3,0];
norm Q^{3,3} a4 = [1,1,0;0,1,1;1,0,1];
square Q^{3,3} a5 = [9,-3,7;5,2,-6;-3/4,2,6];
diag Q^{3,3} a6 = [0,0,0;0,0,0;0,0,0];
psd Q^{3,3} a7 = [2,-1,0;-1,2,-1;0,-1,2];
nsd Q^{3,3} a8 = [-2,1,0;1,-2,1;0,1,-2];

//right operations
herm Q^{3,3} c1 = a1 + a1;
herm Q^{3,3} c2 = a1 - a1;
herm Q^{3,3} c3 = 2.5 * a1;
herm Q^{3,3} c4 = a1^3;
herm Q^{3,3} c5 = a1^(-1);
diag Q^{3,3} c6 = a2 * a2;
square Q^{3,3} c7 = a1 + [0,1,2;3,4,5;6,7,8];
//diag Q^{3,3} c8 = a2';
psd Q^{3,3} c9 = a7 + a7;
psd Q^{3,3} c10 = a1 + a7;
psd Q^{3,3} c11 = a2 * a2;

//right assignments
c1 += c2;
c2 = c1;
c2 = c2 + c1;
c2 = [2,1,0;1,2,0;0,0,2];
c6 -= a6 + a2;
c7 *= c7;

Q d1 = 1;
Q d2 = 2;
B d3 = d1 <= d2;
B d4 = d1 < d2;
B d5 = d2 >= d1;
B d6 = d2 > d1;
B d7 = d1 != d2;
//B d8 = d3 && d4;
//B d9 = d3 || d4;

Q^{1,3} e1 = a1(1,:);


for n = 1:2
    if n < 2
        d1 = 1;
    elseif n < 3
        d1 = 1;
    else
        d1 = 2;
    end
end


end
