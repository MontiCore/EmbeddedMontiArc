// (c) https://github.com/MontiCore/monticore 
function names = getNextBlockNames(blockPath)
    b = get_param(blockPath, 'PortConnectivity');
    handles = [b.DstBlock];
    names = get(handles, 'Name');
end
