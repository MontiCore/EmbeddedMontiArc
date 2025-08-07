// (c) https://github.com/MontiCore/monticore 
function result = getTimeForBlock(blockPath, ind, eoFile)
    if ~exist('eoFile', 'var')
        eoFile = '';
    end
    
    persistent timeDB;
    
    type = getBlockType(blockPath);
    switch type
        case 'VERSION_INFO'
            time = [0 0];
        case 'SubSystem'
            % check if timeDB is present
            if size(timeDB,2) > 0
                % if present, load timeDB
                handles = timeDB(:,1);
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
                bcet = timeDB(timeDB(:,1) == handle, 2);
                wcet = timeDB(timeDB(:,1) == handle, 3);

                time = [bcet wcet];
            else
                % ... if not, compute and store them in timeDB                
                % execution order given, pass it to WCET-method
                time = [getBCETForComponent(blockPath) getWCETForComponent(blockPath,eoFile)];    

                if isempty(time)
                    % if the calculation fails for some reason (e.g. if the
                    % subsystem does not have any outports) try again in
                    % SumOfBlocks mode 
                    time = [getBCETForComponent(blockPath, 'SumOfBlocks') getWCETForComponent(blockPath, eoFile, 'SumOfBlocks')];
                end
                
                % store the computed times in timeDB
                timeDB = appendMatrix(timeDB, [handle time]);
            end
        case 'Inport'
            time = [0 0];
        case 'Outport'
            time = [0 0];
        case 'Constant'
            time = [0 0];
        case 'Terminator'
            time = [0 0];
        case 'Logic'
            time = [1 1];
        case 'RelationalOperator'
            time = [1 1];
        case 'Switch'
            time = [1 2];
        case 'UnitDelay'
            time = [1 1];
        case 'Sum'
            time = [2 2];
        case 'ActionPort'
            time = [0 0];
        case 'DataStoreRead'
            time = [0 0];
        case 'DataStoreWrite'
            time = [0 0];
        case 'Merge'
            time = [1 1];
        case 'BusCreator'
            time = [0 0];
        case 'BusSelector'
            time = [0 0];
        case 'If'
            time = [1 1];
        case 'TriggerPort'
            time = [0 0];
        case 'Math'
            time = [1 4];
        case 'EnablePort'
            time = [0 0];
        case 'Gain'
            time = [2 2];
        otherwise
            fprintf('WARNING: Execution time of BlockType \"%s\" unknown.\n', type);
            time = [1 1];
    end
    
    if nargin < 2
        result = time;
    else
        result = time(ind);
    end
end

        
