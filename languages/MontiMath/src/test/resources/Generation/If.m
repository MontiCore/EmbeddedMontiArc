// (c) https://github.com/MontiCore/monticore 

package Generation;

script If
    Q cond = 2;
    Q result = 0;

    if cond == 1
      result = 1;
    elseif cond == 0
      result = -1;
    else
      result = 0;
    end
end
