// (c) https://github.com/MontiCore/monticore 
classdef NFPCalculator
    %AbstractNFPCalculator  
    %   Detailed explanation goes here
    
    properties (Access = public)
        nfp@AbstractNFP
    end
    
    methods
        function obj = NFPCalculator(nfpObject)  
            obj.nfp = nfpObject;
        end
        
        function result = getNFPForComponent(obj, path, slistFile, mode)
            if ~exist('mode', 'var')
                mode = 'PathLength';
            end
            if ~exist('slistFile', 'var')
                slistFile = '';
            else
                Util.checkEOFile(slistFile);
            end

            switch mode
                case 'SumOfBlocks'
                    if strcmp(Util.getBlockType(path), 'SubSystem')
                        nfp = obj.getSumOfNFPForComponent(path);
                    else 
                        nfp = obj.getNFPForBlock(path);
                    end

                    result = nfp;
                case 'PathLength'
                    outports = Util.getComponentOutputs(path);

                    if isempty(outports)
                        % if no outports could be found, just sum up the blocks of
                        % the component
                        result = obj.getNFPForComponent(path, slistFile, 'SumOfBlocks');              
                    else
                        nfps = [];
                        for i = 1:numel(outports)
                            port = outports(i);
                            paths = Util.getPathsToBlock2(port);

                            for j = 1:size(paths,1)                        
                                tmp = obj.getPathNFP(paths(j,:), slistFile);                       

                                % TODO: preallocate this for speed
                                nfps(size(nfps,1)+1,1) = tmp;
                            end
                        end
                        
                        result = max(nfps);   
                        
                        subComponents = Util.getSubcomponents(path);
                        for j = 1:numel(subComponents)
                           subC = subComponents(j);
                           if strcmp(Util.getBlockName(subC), Util.getBlockName(path))
                               continue
                           end
                           if strcmp(Util.getBlockType(subC), 'SubSystem')
                               pCon = get_param(subC, 'PortConnectivity');
                               if iscell(pCon)
                                   pCon = pCon{1};
                               end
                               if isfield(pCon, 'DstBlock')
                                   dBlocks = {pCon.DstBlock};
                                   dBlocks = dBlocks(~cellfun('isempty',dBlocks));

                                   if isempty(dBlocks)
                                       result = result + obj.getNFPForBlock(subC);
                                   end
                               end
                           end
                        end
                    end
                otherwise
                    fprintf('Unexpected calculation method.\n');
            end
        end
    end
    
    methods (Access = private)
        function value = getNFPForBlock(obj, blockPath, eoFile)
            if ~exist('eoFile', 'var')
                eoFile = '';
            end
            if iscell(blockPath)
                blockPath = blockPath{1};
            end
    
            persistent nfpDB;            

            type = Util.getBlockType(blockPath);
            
            if strcmp(type, 'SubSystem')
               % check if nfpDB is present
                if size(nfpDB,2) > 0
                    % if present, load nfpDB
                    handles = nfpDB(:,1);
                else
                    % else initialize nfpDB
                    handles = [];
                end        

                % get the SubSystem's handle
                handle = get_param(blockPath, 'Handle');

                if ismember(handle, handles)                    
                    value = nfpDB(nfpDB(:,1) == handle, 2);
                else                    
                    value = obj.getNFPForComponent(blockPath, eoFile);

                    if isempty(value)
                        % if the calculation fails for some reason (e.g. if the
                        % subsystem does not have any outports) try again in
                        % SumOfBlocks mode 
                        value = obj.getNFPForComponent(blockPath, eoFile, 'SumOfBlocks');
                    end

                    % store the computed times in energyDB
                    nfpDB = Util.appendMatrix(nfpDB, [handle value]);
                end

            else            
                value = obj.nfp.getValueForBlock(blockPath);
            end
        end
        
        function nfpSum = getSumOfNFPForComponent(obj, path)
            if strcmp(path(length(path)),'/')
                path(length(path)) = '';
            end

            %iterate over subcomponents
            subComponents = find_system(path, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'type', 'block');    

            currNFP = 0;
            for i = 1:numel(subComponents)
                if iscell(subComponents)
                    blockPath = subComponents{i};
                else
                    blockPath = subComponents(i);
                end
                name = getBlockName(blockPath);
                
                if ~strcmp(path, blockPath)
                    tmp = obj.getNFPForBlock(blockPath);
                    if(currNFP == 0)
                        currNFP = tmp;
                    else
                        currNFP = obj.nfp.aggregationOperator(currNFP, tmp);
                    end
                end
            end

            nfpSum = currNFP;
        end

        function pathNFPValue = getPathNFP(obj, input, slistFile)       
            % sum up nfp value of each block of the input path
            pathNFP = -1;
            lastIndex = find(input,1,'last');
            for i = 1:lastIndex
                blockNFP = obj.getNFPForBlock(input(i), slistFile);

                if(pathNFP == -1)
                    pathNFP = blockNFP;
                else
                    pathNFP = obj.nfp.aggregationOperator(pathNFP, blockNFP);
                end
            end

            bgNFP = -1;
            if ~strcmp(slistFile, '')
                % if a file was given, add the background nfp values
                orderedHandles = Util.convertFASDiaryEntries(slistFile);
                m = ismember(orderedHandles,input);
                indices = find(m == 1);

                try
                    % add nfp of the blocks executed in between
                    start = indices(1)+1;
                    ende = indices(size(indices,1))-1;
                    for i = start:ende
                        cBlock = orderedHandles(i);

                        if ismember(cBlock,input)
                            continue
                        end

                        tmpNFP = obj.getNFPForBlock(orderedHandles(i));
                        if(bgNFP == -1)
                            bgNFP = tmpNFP;
                        else
                            bgNFP = obj.nfp.aggregationOperator(bgNFP,tmpNFP);
                        end
                    end
                catch
                    % no matches in orderedHandles
                end
            end

            if(bgNFP == -1)
                pathNFPValue = pathNFP;
            else
                pathNFPValue = obj.nfp.aggregationOperator(pathNFP, bgNFP);
            end
        end
    end
end

