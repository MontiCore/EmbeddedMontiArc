// (c) https://github.com/MontiCore/monticore 
function time = getBCPathTime(input)
    lastIndex = find(input,1,'last');
    
    time = 0;
    for i = 1:lastIndex
        blockTime = getTimeForBlock(input(i));
        
        time = time + blockTime(1);
    end
end

