// (c) https://github.com/MontiCore/monticore 
function outputString = tagSubsystem(sys, nfpCalc, slistFile, ind, parentPath)
    tagValue = nfpCalc.getNFPForComponent(sys, slistFile);
    compName = Util.getBlockName(sys);
    compName = [lower(compName(1)) compName(2:end)];
    compType = Util.getBlockType(sys);
    
    if ~isempty(tagValue) && ~(strcmp(compType, 'VERSION_INFO')) && ~(strcmp(compType, 'SysInit'))
        tagValue = num2str(tagValue);
        tagType = getOutputTagType(nfpCalc.nfp.name);
        if ~strcmp(parentPath, '')
            parentPath = [parentPath '.'];
        end
        pathString = [parentPath compName];
        outputUnit = nfpCalc.nfp.unit;
        
        outputString = ['\n' ind 'tag ' pathString ' with ' tagType ' = ' tagValue ' ' outputUnit ';\n'];
                
        % Uncomment following lines for hierarchical output using {}-brackets
        % outputString = ['\n' ind 'tag ' char(getComponent(sys)) ' with ' tagType ' = ' tagValue ' {\n'];
        % ind = indent(ind);
        
        %iterate over subcomponents
        subComponents = getSubcomponents(sys);
        types = Util.getBlockType(subComponents);
        for i = 1:numel(subComponents)        
            blockPath = subComponents{i};
            type = types{i};

            if ~strcmp(sys, blockPath)
                %Subsystem
                if isSubsystem(blockPath)
                    outputString = [outputString tagSubsystem(blockPath, nfpCalc, slistFile, ind, pathString)];
                else
                    outputString = [outputString tagAtomicBlock(blockPath, nfpCalc, slistFile, ind, pathString)];
                end
            end
        end

        % Uncomment when using hierarchical output using {}-brackets;
        % ind = unindent(ind);
        % outputString = [outputString ind endTagModel];
        outputString = [outputString '\n'];
    else
        outputString = '';
    end
end
