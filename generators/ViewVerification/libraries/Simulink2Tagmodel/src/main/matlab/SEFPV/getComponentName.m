// (c) https://github.com/MontiCore/monticore 
function name = getComponentName(path)
%GETCOMPONENTNAME returns the name for a component.
%   path is a path separated by '/' to a component. If the path looks like
%   'SubSystem/.../SubSystem/Component' then 'component' is returned in
%   name. Note that the returned name starts with a lower case letter.
%   If the component name contains characters other than letters, numbers
%   or underscore, they are removed. If the name starts with a number, an
%   underscore is prefixed.
    if iscell(path)
        name = cell(length(path), 1);
        for i = 1:length(path)
            name{i} = getComponentName(path{i});
        end
    else
        component = getComponent(path);
        name = changeFirstCharacterCase(component, true);
    end
end
