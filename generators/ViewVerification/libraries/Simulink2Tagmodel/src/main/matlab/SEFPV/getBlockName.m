// (c) https://github.com/MontiCore/monticore 
function name = getBlockName( path )
%GETBLOCKTYPE Summary of this function goes here
%   Detailed explanation goes here
    if iscell(path)
        s = size(path);
        name = cell(s);
        for i = 1:s
            name{i} = getBlockName(path{i});
        end
    else
        if hasParam(path, 'Name')
            name = get_param(path, 'Name');
        elseif strcmp(get_param(path, 'Name'), 'block_diagram')
            name = 'SubSystem';
        end
    end
    
    % remove whitespaces from names
    name = strrep(name, ' ', '');
end

