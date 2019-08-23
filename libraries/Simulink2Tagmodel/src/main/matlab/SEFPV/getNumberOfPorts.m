// (c) https://github.com/MontiCore/monticore 
function numOfPorts = getNumberOfPorts( blockPath, b )
%GETNUMBEROFPORTS Summary of this function goes here
%   Detailed explanation goes here
    if hasParam(blockPath, 'PortHandles')
        ph = get_param(blockPath, 'PortHandles');

        if b
            numOfPorts = length(ph.Inport);
        else
            numOfPorts = length(ph.Outport);
        end
    elseif isSubsystem(blockPath)
        if b
            iotype = 'Inport';
        else
            iotype = 'Outport';
        end
        portNames = find_system(blockPath, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', iotype);
        numOfPorts = numel(portNames);
    else
        numOfPorts = 0;
    end
end

