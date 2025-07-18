// (c) https://github.com/MontiCore/monticore 
function sys = expandMask(path)
    if isMask(path)
        set_param(path, 'Mask', 'off');
        Simulink.BlockDiagram.expandSubsystem(path);
    end
end
