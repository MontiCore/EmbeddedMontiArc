// (c) https://github.com/MontiCore/monticore 
function handles = getInputHandles(blockPath)
    pCon = get_param(blockPath, 'PortConnectivity');
    inputs = {pCon.SrcBlock};
    inputs = inputs(~cellfun('isempty',inputs));
    
    handles = inputs;
end

