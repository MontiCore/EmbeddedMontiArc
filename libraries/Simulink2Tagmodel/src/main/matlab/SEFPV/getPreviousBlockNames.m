// (c) https://github.com/MontiCore/monticore 
function names = getPreviousBlockNames(blockPath)
    b = get_param(blockPath, 'PortConnectivity');
    handles = [b.SrcBlock];
    names = get(handles, 'Name');
end
