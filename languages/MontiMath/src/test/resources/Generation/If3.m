// (c) https://github.com/MontiCore/monticore 

package Generation;

script If3
  Q cond1 = 1;
  Q cond2 = -1;
  Q result = 0;

  B bool = cond1 == cond2;

  if bool
    result = 1;
  elseif bool || cond2 < 0
    result = -1;
  else
    result = 0;
  end
end
