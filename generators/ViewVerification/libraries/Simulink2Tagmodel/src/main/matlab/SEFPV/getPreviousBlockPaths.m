// (c) https://github.com/MontiCore/monticore 
function paths = getPreviousBlockPaths(blockPath)
    names = getPreviousBlockNames(blockPath);
    
    if isempty(names)
        paths = {};
    else
        parts = strsplit(blockPath,'/');
        prefix = '';

        if length(parts) > 1 && strcmp(parts{length(parts)}, '')
            parts(length(parts)) = {};
        end

        parts(length(parts)) = [];
        prefix = strjoin(parts, '/');

        paths = strcat(prefix,'/',names);
    end
end
