// (c) https://github.com/MontiCore/monticore 
function wcet = getWCETForComponent(path, slistFile, mode)
    if ~exist('mode', 'var')
        mode = 'PathLength';
    end
    
    eoFileGiven = false;
    if exist('slistFile', 'var')
       if ~strcmp(slistFile, '')
          try
              f = fopen(slistFile);
              if ~(f == -1)
                 eoFileGiven = true; 
              end
              fclose(f);
          catch
              fprintf('Cannot open file. Please check the file name.\n');
          end
       end
    end
    
    switch mode
        case 'SumOfBlocks'
            time = 0;

            if strcmp(getBlockType(path), 'SubSystem')
                time = getSumOfBlocksForSubsystem(path, true);
            else 
                time = getTimeForBlock(path, 2);
            end

            wcet = time;
        case 'PathLength'
            outports = getComponentOutputs(path);
            
            if isempty(outports)
                % if no outports could be found, just sum up the blocks of
                % the component
                wcet = getWCETForComponent(path,'','SumOfBlocks');              
            else
                times = [];
                for i = 1:numel(outports)
                    port = outports(i);
                    paths = getPathsToBlock(port);
                    n = getBlockNames(paths);

                    for j = 1:size(paths,1)
                        tmp = 0;
                        if eoFileGiven
                            %
                            % TODO: check if this is fixed - 04.04.2017
                            %
                            % currently not working realiably for
                            % components with subsystems, because subsystems 
                            % like "RSFlipFlop" do not appear in the EO
                            %
                            % => compute paths by expanding them?
                            tmp = getWCPathTimeFromEO(paths(j,:), slistFile);
                        else
                            tmp = getWCPathTime(paths(j,:));
                        end
                        
                        times(size(times,1)+1,1) = tmp;
                    end
                end

                wcet = max(times);   
            end
        otherwise
            fprintf('Unexpected calculation method.\n');
    end
end
