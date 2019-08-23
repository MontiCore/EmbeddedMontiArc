// (c) https://github.com/MontiCore/monticore 
function indentation = unindent(s)
%UNINDENT unindents the string s by deleting from the beginning of the 
%string.
    if length(s) > 2
        indentation = s(3:end);
    else
        indentation = '';
    end
end 
