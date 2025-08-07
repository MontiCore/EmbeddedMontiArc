// (c) https://github.com/MontiCore/monticore 
function m = appendMatrix(m1,m2)
    s11 = size(m1,1);
    s12 = size(m1,2);
    s21 = size(m2,1);
    s22 = size(m2,2);
    
    m1(s11+1:(s11+s21),1:s22) = m2;
    
    m = m1;
end
