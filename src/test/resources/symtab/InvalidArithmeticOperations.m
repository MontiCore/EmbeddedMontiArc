// (c) https://github.com/MontiCore/monticore 

package symtab;

script InvalidArithmeticOperations

Q^{1,2} a = [1 , 2];
Q^{1,3} b = [1,2,3];
c = a + b;

a = [2,3;4,5;6,7]*[1,2;3,4;5,6];
b = [1,2;3,4;5,6]^2;

end
