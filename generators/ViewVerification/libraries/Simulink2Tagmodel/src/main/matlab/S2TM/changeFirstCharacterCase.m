// (c) https://github.com/MontiCore/monticore 
function outputString = changeFirstCharacterCase( string, toLower )
%CHANGEFIRSTCHARACTERCASE changes the first character of the string to
%lower or to upper case depending on the boolean value of the second
%arguemnt.

    if ~isempty(string)
        if toLower
            outputString = [lower(string(1)) string(2:end)];
        else
            outputString = [upper(string(1)) string(2:end)];
        end
    else
        outputString = '';
    end
end

