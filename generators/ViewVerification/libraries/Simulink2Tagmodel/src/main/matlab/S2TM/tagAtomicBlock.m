// (c) https://github.com/MontiCore/monticore 
function outputString = tagAtomicBlock(blockPath, nfpCalc, slistFile, ind, parentPath)
    type = Util.getBlockType(blockPath);
    name = Util.getBlockName(blockPath);
    name = [lower(name(1)) name(2:end)];
    
    if strcmp('Inport', type) || strcmp('Outport', type) || strcmp('Constant', type) ...
            || strcmp('ActionPort', type) || strcmp('EnablePort', type) || strcmp('TriggerPort', type) ...
            || strcmp('BusCreator', type) || strcmp('BusSelector', type) || strcmp('VERSION_INFO', type) ...
            || strcmp('DataStoreWrite', type) || strcmp('DataStoreRead', type) || strcmp('Merge', type) ...
            || strcmp('SysInit', type)
        outputString = '';
    else        
        % compute tag according to given type
        tag = nfpCalc.getNFPForComponent(blockPath, slistFile);
        if ~strcmp(parentPath, '')
            parentPath = [parentPath '.'];
        end
        
        outputPref = [ind 'tag ' parentPath name ' with '];
        outputTagType = getOutputTagType(nfpCalc.nfp.name);       
        outputUnit = nfpCalc.nfp.unit;    
            
        outputString = [outputPref outputTagType ' = ' num2str(tag) ' ' outputUnit '; \n'];
    end
end

        
