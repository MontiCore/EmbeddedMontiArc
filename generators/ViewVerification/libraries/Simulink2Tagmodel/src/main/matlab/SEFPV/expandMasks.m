// (c) https://github.com/MontiCore/monticore 
function sys = expandMasks(componentPath)
    subComponents = find_system(componentPath, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'type', 'block');
   
    for i = 1:numel(subComponents)
        if isSubsystem(subComponents{i}) & isMask(subComponents{i})
            set_param(subComponents{i}, 'Mask', 'off');
            Simulink.BlockDiagram.expandSubsystem(subComponents{i});
        end
    end
end
