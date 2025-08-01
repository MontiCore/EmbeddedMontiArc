// (c) https://github.com/MontiCore/monticore 
function cleanedString = replaceForbiddenCharacters( string )
%REPLACEFORBIDDENCHARACTERS Removes characters from the provided string
%that are not allowed in MontiArc names.
%   Characters other than letters, numbers and underscore are prohibited in
%   MontiArc identifiers. Hence, they are removed. Moreover, names in
%   MontiArc cannot begin with a number so an underscore is prefixed in
%   that case.

    %delete any non letter, numeric or underscore character
    string = regexprep(string, '[^a-zA-Z0-9_]*', '');
    
    %if the string does not start with a letter or underscore, add _ as
    %first letter
    if regexp(string, '^[^a-zA-Z_].*$')
        string = ['_' string];
    end
    
    cleanedString = string;
end

