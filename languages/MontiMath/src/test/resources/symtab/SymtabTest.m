// (c) https://github.com/MontiCore/monticore 

package symtab;

script SymtabTest

Q a = 2+7*5+4%3;
Q^{2,2} m1 = [23, 44; 22, 222] + [ 1, 2; 3,4] * [3,4;5,6] - [7,8;9,10].*[22,1;2,3];
Q b = -7;
Q c = 9;
Q d = b + c;
Q h = 2 km + 7 m * 3 m^3; // unit error
Q e = 7 + d * b;
//Q^{2,2} m2 = [1,2;4+3*2+1,4];
Q f = e * b + c * m2(2,1);
Q(0 : 10 km^2) g = 2 km + (7 m * 3 m)^(3); // unit error
Q(1 mm : 1 cm : 10 km)^{1,3} i1 = [ 2 mm, 3 cm,4 km ]; // i is reserved for complex numbers
Q(0 : 10 km)^{3,1} j = [ 2 mm; 3 cm;4 km ];
Q^{2,3} k = ( [1,1;1,1;1,1]+ [1,1;1,1;1,1] ) ';
Q^{2,2} l = [1,2;3,4]^4;
Q^{4} vec = 2:2:8;
Q^3 vec1 = [1;1;1];
Q^{3,3} matrix1 = [1,2,3;4,5,6;7,8,9];
Q^{2,3} matrix2 = matrix1(1:2,:);
Q^{3} matrix3 = matrix1(3, :);
Q(0 : 10 km)^{1,3} i2 = [ 2 mm, 3 cm,4 km ];
Q(0 : 10 km)^{3,1} j2 = [ 2 mm; 3 cm;4 km ];
B bool = j2 == i2'; // true if j is equals i transpose

Q test = degree^(-0.5) * similarity * degree^(-0.5);
end
