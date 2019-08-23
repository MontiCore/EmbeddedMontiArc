// (c) https://github.com/MontiCore/monticore 
function time = getWCPathTimeFromEO(input, slistFile)
    orderedHandles = convertDiaryEntries(slistFile);
    m = ismember(orderedHandles,input);
    indices = find(m == 1);
    time = 0;
    
    % sum up times of each block of the input path
    pathTime = 0;
    lastIndex = find(input,1,'last');
    for i = 1:lastIndex
        blockTime = getTimeForBlock(input(i),2,slistFile);
        
        pathTime = pathTime + blockTime;
    end
    
    bgTime = 0;
    try
        % add times of the blocks executed in between
        start = indices(1)+1;
        ende = indices(size(indices,1))-1;
        for i = start:ende
            cBlock = orderedHandles(i);

            if ismember(cBlock,input)
                continue
            end
            
            tmpTime = getTimeForBlock(orderedHandles(i),2);
            bgTime = bgTime + tmpTime;
        end
    catch
        % no matches in orderedHandles
    end
    
    time = pathTime + bgTime;
end

