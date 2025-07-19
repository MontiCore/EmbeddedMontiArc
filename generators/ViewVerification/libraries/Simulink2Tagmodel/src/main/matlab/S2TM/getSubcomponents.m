// (c) https://github.com/MontiCore/monticore 
function subComponents = getSubcomponents(path)
    subComponents = find_system(path, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'type', 'block');
end

