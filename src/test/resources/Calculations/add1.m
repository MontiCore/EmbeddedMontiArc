// (c) https://github.com/MontiCore/monticore 

package calculations;

script add1
    Q^{1,2} A = [1,2] + [3,4];
    A = [(1+2),4];
    Q^{2,3} b = [1,2,4;5,6,7];
    A = [1,2]./[2,3];
    Q^{2,1} c = [1,2].';
    A = [1+2*4;8];
    Q d = A(1,0);
    B e = false;
end
