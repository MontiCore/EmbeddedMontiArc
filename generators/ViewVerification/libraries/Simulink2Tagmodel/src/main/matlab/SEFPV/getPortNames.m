// (c) https://github.com/MontiCore/monticore 
function ports = getPortNames(blockPath, b)
%GETIOPORTNAMES returns the port names of a component.
%   blockPath is a Simulink path to a component and b is a logical variable
%   specifying whether to return the Inport or Outport names. ports is a
%   cell array containing the port names (that might be '').
    if b
        iotype = 'Inport';
    else
        iotype = 'Outport';
    end
    
    if strcmp(getBlockType(blockPath), 'SubSystem')
        portNames = find_system(blockPath, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', iotype);
        portNumbers = get_param(portNames, 'Port');
        ports = cell(size(portNames));
        for i = 1:length(portNames)
            ports{i} = renamePort(getBlockType(blockPath), portNames{i}, portNumbers{i}, b);
        end
        
        if b
            if isEnabledSubsystem(blockPath)
                ports{length(ports)+1} = renamePort('SubSystem', 'enable', '', true);
            elseif isTriggeredSubsystem(blockPath)
                ports{length(ports)+1} = renamePort('SubSystem', 'trigger', '', true);
            elseif isIfActionSubsystem(blockPath)
                ports{length(ports)+1} = renamePort('SubSystem', 'ifaction', '', true);
            end
        end
    else
        ports = renamePorts(getBlockType(blockPath), getfield(get_param(blockPath, 'PortHandles'), iotype), b);
    end
    
    ports = cellstr(ports);
end

