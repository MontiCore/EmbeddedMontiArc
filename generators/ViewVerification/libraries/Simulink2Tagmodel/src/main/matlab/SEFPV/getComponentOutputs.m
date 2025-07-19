// (c) https://github.com/MontiCore/monticore 
function ports = getComponentOutputs(blockPath)
    ports = [];
    
    if strcmp(getBlockType(blockPath), 'SubSystem')
        portNames = find_system(blockPath, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'Outport');
        portNumbers = get_param(portNames, 'Handle');
        
        if iscell(portNumbers)
            ports = cell2mat(portNumbers);
        else
            ports = portNumbers;
        end
    else
        ports = [];
    end
    
    if size(ports) == 0
       % blockPath is a SubSystem, but has no outports
       % instead, look for Data Store Write blocks. 
       dsw = find_system(blockPath, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'DataStoreWrite');
       portNumbers = get_param(dsw, 'Handle');
        
       if iscell(portNumbers)
           ports = cell2mat(portNumbers);
       else
           ports = portNumbers;
       end
    end
end

