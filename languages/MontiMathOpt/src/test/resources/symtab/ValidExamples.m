// (c) https://github.com/MontiCore/monticore 

package symtab;

script ValidExamples
//Z (0 : 1 ) test = 0;
Q a =  2+7*5+4%3;
Q^{2,2} m1 = [23, 44; 22, 222] + [ 1, 2; 3,4] * [3,4;5,6] - [7,8;9,10].*[22,1;2,3];
Q b = -7;
Q c = 9;
Q d = b + c;
Q e = 7 + d * b;
Q^{2,2} m2 = [1,2;3*4+3,4];
Q f = e * b + c * m2(1,0);
Q(0 : 10 km)^{1,3} i = [ 2 mm, 3 cm,4 km ];
Q(0 : 10 km)^{3,1} j = [ 2 mm; 3 cm;4 km ];
B bool = j == i'; // true if j is equals i transpose
Z(0:2)^{2,3} k = ( [1,1;1,1;1,1]+ [1,1;1,1;1,1] )';
Z^{2,2} l = [1,2;3,4]^4;
Z(2:8)^{4} vec = 2:2:8;
Z^{3,3} matrix1 = [1 2 3;4 5 6;7 8 9];
Z^{2,3} matrix2 = matrix1(1:2, 1:3);
Z^{8} v = [16 5 9 4 2 11 7 14];
Z^{4} v1 = v(1:2:8);

end
