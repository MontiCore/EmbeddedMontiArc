// (c) https://github.com/MontiCore/monticore 
function energy = getPathEnergy(input, slistFile)       
    % sum up energy of each block of the input path
    pathEnergy = 0;
    lastIndex = find(input,1,'last');
    for i = 1:lastIndex
        blockEnergy = getEnergyForBlock(input(i),slistFile);

        pathEnergy = pathEnergy + blockEnergy;
    end

    bgEnergy = 0;
    if ~strcmp(slistFile, '')
        % if a file was given, add the background energy
        orderedHandles = convertDiaryEntries(slistFile);
        m = ismember(orderedHandles,input);
        indices = find(m == 1);
        
        try
            % add energy of the blocks executed in between
            start = indices(1)+1;
            ende = indices(size(indices,1))-1;
            for i = start:ende
                cBlock = orderedHandles(i);

                if ismember(cBlock,input)
                    continue
                end

                tmpEnergy = getEnergyForBlock(orderedHandles(i));
                bgEnergy = bgEnergy + tmpEnergy;
            end
        catch
            % no matches in orderedHandles
        end
    end
    
    energy = pathEnergy + bgEnergy;
end

