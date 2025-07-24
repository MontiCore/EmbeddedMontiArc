// (c) https://github.com/MontiCore/monticore 
function energy = getSumOfEnergyForComponent(path)
    if strcmp(path(length(path)),'/')
        path(length(path)) = '';
    end
    
    %iterate over subcomponents
    subComponents = find_system(path, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'type', 'block');    
    
    currentEnergy = 0;
    for i = 1:numel(subComponents)
        blockPath = subComponents{i};
        
        if ~strcmp(path, blockPath)
            tmp = getEnergyForBlock(blockPath);
            currentEnergy = currentEnergy + tmp;
        end
    end
    
    energy = currentEnergy;
end
