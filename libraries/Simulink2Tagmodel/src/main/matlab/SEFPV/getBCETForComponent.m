// (c) https://github.com/MontiCore/monticore 
function bcet = getBCETForComponent(path, mode)
    if ~exist('mode', 'var')
        mode = 'PathLength';
    end
    
    switch mode
        case 'SumOfBlocks'
            time = 0;

            if strcmp(getBlockType(path), 'SubSystem')
                time = getSumOfBlocksForSubsystem(path, false);
            else 
                time = getTimeForBlock(path, 1);
            end

            bcet = time;
        case 'PathLength'
            outports = getComponentOutputs(path);
            
            if isempty(outports)
                % if no outports could be found, just sum up the blocks of
                % the component
                bcet = getBCETForComponent(path,'SumOfBlocks');
            else
                times = [];
                for i = 1:numel(outports)
                    port = outports(i);
                    paths = getPathsToBlock(port);
                    tmpTimes = [];                

                    for j = 1:size(paths,1)                        
                        tmpTimes(j,1) = getBCPathTime(paths(j,:));                        
                    end

                    times(i,1) = min(tmpTimes);
                end

                bcet = max(times);   
            end
        otherwise
            fprintf('Unexpected calculation method.\n');
    end
end
