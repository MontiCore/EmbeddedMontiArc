// (c) https://github.com/MontiCore/monticore 
function result = getEnergyForBlock(blockPath, eoFile)
    if ~exist('eoFile', 'var')
        eoFile = '';
    end
    
    persistent energyDB;
    
    type = getBlockType(blockPath);
    switch type
        case 'VERSION_INFO'
            energy = 0;
        case 'SubSystem'
            % check if energyDB is present
            if size(energyDB,2) > 0
                % if present, load timeDB
                handles = energyDB(:,1);
            else
                % else initialize timeDB
                handles = [];
            end        
            
            % get the SubSystem's handle
            handle = get_param(blockPath, 'Handle');

            % check if execution times have already been computed for 
            % that handle...
            if ismember(handle, handles)
                % ... if yes, load them ...
                energy = energyDB(energyDB(:,1) == handle, 2);
            else
                % ... if not, compute and store them in timeDB
                energy = getEnergyForComponent(path, eoFile);

                if isempty(energy)
                    % if the calculation fails for some reason (e.g. if the
                    % subsystem does not have any outports) try again in
                    % SumOfBlocks mode 
                    energy = getEnergyForComponent(path, eoFile, 'SumOfBlocks');
                end
                
                % store the computed times in energyDB
                energyDB = appendMatrix(energyDB, [handle energy]);
            end
        case 'Inport'
            energy = 0;
        case 'Outport'
            energy = 0;
        otherwise
            fprintf('WARNING: Energy of BlockType \"%s\" unknown.\n', type);
            energy = 0;
    end
    
    result = energy;
end

        
