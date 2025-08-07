// (c) https://github.com/MontiCore/monticore 
function time = getSumOfBCETForSubsystem(path)
    if strcmp(path(length(path)),'/')
        path(length(path)) = '';
    end
    
    %iterate over subcomponents
    subComponents = find_system(path, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'type', 'block');
    types = getBlockType(subComponents);    
    
    currentTime = 0;
    for i = 1:numel(subComponents)
        blockPath = subComponents{i};
        
        if iscell(types)
            type = types{i};
        else
            type = types;
        end
        
        if ~strcmp(path, blockPath)
            tmp = getTimeForBlock(blockPath,1);
            currentTime = currentTime + tmp;
        end
    end
    
    time = currentTime;
end
