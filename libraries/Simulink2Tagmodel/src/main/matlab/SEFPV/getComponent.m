// (c) https://github.com/MontiCore/monticore 
function name = getComponent(path)
%GETCOMPONENT returns the name with a capital first letter of a component.
%   path is a path separated by '/' to a component. If the path looks like
%   'SubSystem/.../SubSystem/Component' then 'Component' is returned in
%   name. Sepcial characters other than underscore are removed from the
%   name. If it solely contains of a java keyword followed by numbers, the
%   string 'Block' is inserted. 
    if ~isempty(path)
        n = strsplit(path, '/');
        if length(n) > 1 && strcmp(n{length(n)}, '')
            name = n{length(n)-1};
        else
            name = n{length(n)};
        end
        name = replaceForbiddenCharacters(name);
        
        javaKeywords = {'abstract'; 'continue'; 'for'; 'new'; 'switch'; ...
            'assert'; 'default'; 'goto'; 'package'; 'synchronized'; ...
            'boolean'; 'do'; 'if'; 'private'; 'this'; 'break'; 'double'; ...
            'implements'; 'protected'; 'throw'; 'byte'; 'else'; 'import'; ...
            'public'; 'throws'; 'case'; 'enum'; 'instanceof'; 'return'; ...
            'transient'; 'catch'; 'extends'; 'int'; 'short'; 'try'; ...
            'char'; 'final'; 'interface'; 'static'; 'void'; 'class'; ...
            'finally'; 'long'; 'strictfp'; 'volatile'; 'const'; 'float'; ...
            'native'; 'super'; 'while'};
        for i = 1:numel(javaKeywords)
            keyword = javaKeywords{i};
            %check if name starts with a java keyword followed only by
            %numbers
            if ~isempty(regexpi(name, ['^' keyword '\d*$']))
                %insert 'Block' between keyword and numbers
                name = regexprep(name, [keyword '(?=\d*$)'], [name(1:length(keyword)) 'Block'], 'ignorecase');
                break;
            end
        end
        name = changeFirstCharacterCase(name, false);
    else
        name = '';
    end
end
