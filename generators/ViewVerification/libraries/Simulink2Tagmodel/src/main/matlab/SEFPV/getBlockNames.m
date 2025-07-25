// (c) https://github.com/MontiCore/monticore 
function names = getBlockNames(blocks)
    names = {};
    
    for i = 1:size(blocks,1)
        for j = 1:size(blocks,2)
            handle = blocks(i,j);
            
            if handle == -1
                names(i,j) = {'LOOPING PATH!'};
                continue
            end
            
            if handle == 0
                names(i,j) = {''};
                continue
            end
            
            name = get_param(blocks(i,j), 'Name');
            names(i,j) = {name};
        end
    end
end

