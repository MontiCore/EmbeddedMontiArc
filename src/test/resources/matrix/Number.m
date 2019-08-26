// (c) https://github.com/MontiCore/monticore 

package matrix;

script Number
    Q(0 : 10) n = 7/3;

    Q(0:10) n1 = 5; // this is ok, because 5 is between 0 and 10
    n1 = 15; // the parser should parse it, but there should be a context condition that checks, that 15 is not in the range between 0 and 10
end
