// (c) https://github.com/MontiCore/monticore 
function m = startTagModel(startCmp, nfpCalculator, slistFile, ind)
    name = startCmp;
    parts = strsplit(name, '/');
    
    root = '';
    for i = 1:numel(parts)
        if ~strcmp(root,'')
            root = [root '.'];
        end
        tmp = parts{i};
        tmp = [lower(tmp(1)) tmp(2:end)];
        root = [root tmp];        
    end
    switch(nfpCalculator.nfp.name)
        case {'WCET', 'BCET'}
            m = ['\ntags TagLatency for ' root ' {\n'];
        case 'CompPower'
            m = ['\ntags CompPower for ' root ' {\n'];
        otherwise
            error('Unknown tag type.')
    end
       
    ind = indent(ind);
    subComponents = getSubcomponents(startCmp);
    types = Util.getBlockType(subComponents);
    for i = 1:numel(subComponents)        
        blockPath = subComponents{i};
        name = Util.getBlockName(blockPath);
        type = types{i};
        
        if ~strcmp(startCmp, blockPath)
            %Subsystem
            if isSubsystem(blockPath)
                m = [m tagSubsystem(blockPath, nfpCalculator, slistFile, ind, '')];
            else
                m = [m tagAtomicBlock(blockPath, nfpCalculator, slistFile, ind, '')];
            end
        end
    end
end

