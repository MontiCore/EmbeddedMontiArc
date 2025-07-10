// (c) https://github.com/MontiCore/monticore 
function result = getEnergyForComponent(path, slistFile, mode)
    if ~exist('mode', 'var')
        mode = 'PathLength';
    end
    if ~exist('slistFile', 'var')
        slistFile = '';
    else
        checkEOFile(slistFile);
    end
    
    switch mode
        case 'SumOfBlocks'
            if strcmp(getBlockType(path), 'SubSystem')
                energy = getSumOfEnergyForSubsystem(path);
            else 
                energy = getEnergyForBlock(path);
            end

            result = energy;
        case 'PathLength'
            outports = getComponentOutputs(path);
            
            if isempty(outports)
                % if no outports could be found, just sum up the blocks of
                % the component
                result = getEnergyForComponent(path, slistFile, 'SumOfBlocks');              
            else
                energies = [];
                for i = 1:numel(outports)
                    port = outports(i);
                    paths = getPathsToBlock(port);

                    for j = 1:size(paths,1)                        
                        tmp = getPathEnergy(paths(j,:), slistFile);                       
                        
                        % TODO: preallocate this for speed
                        energies(size(energies,1)+1,1) = tmp;
                    end
                end

                result = max(energies);   
            end
        otherwise
            fprintf('Unexpected calculation method.\n');
    end
end

function checkEOFile(slistFile)
    try
      f = fopen(slistFile);
      fclose(f);
    catch
      error('Cannot open file. Please check the file name.\n');
    end
end
