// (c) https://github.com/MontiCore/monticore 

package matrix;

script MatrixPropertiesNeg

//right declarations
herm square Q^{3,3} a1 = [2,1,0;1,2,0;0,0,2];
diag square Q^{3,3} a2 = [2+1,0,0;0,2/3,0;0,0,2*2];
skewHerm Q^{3,3} a3 = [0,1,0;-1,0,-3;0,3,0];
norm Q^{3,3} a4 = [1,1,0;0,1,1;1,0,1];
square Q^{3,3} a5 = [9,-3,7;5,2,-6;-3/4,2,6];
diag Q^{3,3} a6 = [0,0,0;0,0,0;0,0,0];
psd Q^{3,3} a7 = [2,-1,0;-1,2,-1;0,-1,2];

//wrong declarations
diag Q^{3,3} b1 = [2,0,0;0,2,0;0,1,2];
herm Q^{3,3} b2 = [2,1,0;-2,2,0;0,0,2];
skewHerm Q^{3,3} b3 = [2,1,0;1,2,0;0,0,2];
norm Q^{3,3} b4 = [2,1,4;-1,2,1/2;11,3,2];
square Q^{3,2} b5 = [2,0;0,2;0,0];
square Q^{2,3} b6 = [2,0,0;2,0,0];

//wrong operations
herm Q^{3,3} d1 = a1 * a1;
herm Q^{3,3} d2 = a1 / a1;
diag Q^{3,3} d3 = a1 + a2;
norm Q^{3,3} d4 = a4 * a4;
norm Q^{3,3} d5 = a4 + a4;
norm Q^{3,3} d6 = a4^(-1);
//psd Q^{3,3} d7 = a7';

//wrong assignments
a3 = a1;
a5 = [2,1;1,2;2,1];
a1 = a1 * a1;
a4 += a4;
a1 -= a3;
a4 *= a4;

end
