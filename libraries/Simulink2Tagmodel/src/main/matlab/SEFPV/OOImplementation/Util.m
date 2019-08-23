// (c) https://github.com/MontiCore/monticore 
classdef Util
    %UTIL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Static)
        % Check if the given file path can be openend
        function checkEOFile(slistFile)
            if(strcmp(slistFile,''))
                return
            end
            try
              f = fopen(slistFile);
              fclose(f);
            catch
              error('Cannot open file. Please check the file name.\n');
            end
        end
        
        function subComponents = getSubcomponents(path)
            subComponents = find_system(path, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'type', 'block');
        end
        % Get the outports of the given component
        function ports = getComponentOutputs(blockPath)
            ports = [];

            if strcmp(Util.getBlockType(blockPath), 'SubSystem')
                portNames = find_system(blockPath, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'Outport');
                portNumbers = get_param(portNames, 'Handle');

                if iscell(portNumbers)
                    ports = cell2mat(portNumbers);
                else
                    ports = portNumbers;
                end
            else
                ports = [];
            end

            if size(ports) == 0
               % blockPath is a SubSystem, but has no outports
               % instead, look for Data Store Write blocks. 
               dsw = find_system(blockPath, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'DataStoreWrite');
               portNumbers = get_param(dsw, 'Handle');

               if iscell(portNumbers)
                   ports = cell2mat(portNumbers);
               else
                   ports = portNumbers;
               end
            end
        end
        
        function [paths, blocksVisited] = getPathsToBlock2(path, loopCount, blocksVisited)
            if nargin < 2
                loopCount = 1;
            end
            if nargin < 3
                blocksVisited = [];
            end
            
            paths = [];
            loopMarker = 'LOOP';            
            blockHandle = get_param(path, 'Handle');
            pCon = get_param(path, 'PortConnectivity');
            sBlocks = {pCon.SrcBlock};
            sBlocks = sBlocks(~cellfun('isempty',sBlocks)); 
            
            if isempty(sBlocks)
                paths = blockHandle;
            else     
                tmpPaths = [];
                for i = 1:numel(sBlocks)
                  prevBlock = sBlocks{i};
                   
                  if(size(find(blocksVisited == prevBlock)) == loopCount)
                     paths = Util.appendMatrix(paths, [blockHandle, loopMarker, prevBlock]);
                  else
                     blocksVisited = [blocksVisited; blockHandle];
                     prevPaths = Util.getPathsToBlock2(prevBlock,loopCount,blocksVisited);
                               
                     % TODO: check if prevPath is a LOOP path
                     for j = 1:size(prevPaths,1)
                       prevPaths(j,find(prevPaths(j,:),1,'last')+1) = blockHandle;
                     end
                                          
                     tmpPaths = Util.appendMatrix(tmpPaths, prevPaths);
                  end
                end
                
                paths = tmpPaths;
            end
        end
        
        % Get all possible paths leading from an input (or a constant)
        % to the specified block
        function [paths, blocksVisited] = getPathsToBlock(path, loopCount, blocksVisited)
           if nargin < 2
               % Default loopCount  
               loopCount = 1;
           end
           if nargin < 3
              blocksVisited = []; 
           end

           pCon = get_param(path, 'PortConnectivity');
           sBlocks = {pCon.SrcBlock};
           sBlocks = sBlocks(~cellfun('isempty',sBlocks)); 

           paths = [];

           % mark this block as visited
           blockHandle = get_param(path, 'Handle');
           blocksVisited = [blocksVisited; blockHandle];

           if isempty(sBlocks)
               paths = path;
           else   
               tmpPaths = [];

               for i = 1:numel(sBlocks)
                   previousBlock = sBlocks{i};

                   if ismember(previousBlock,blocksVisited)
                       % ignore loop paths
                       previousPaths = [-1 previousBlock];
                   else
                       previousPaths = Util.getPathsToBlock(previousBlock, loopCount, blocksVisited);
                   end

                   % append current block to previous paths
                   for j = 1:size(previousPaths,1)
                       previousPaths(j,find(previousPaths(j,:),1,'last')+1) = get_param(path, 'Handle');
                   end

                   tmpPaths = Util.appendMatrix(tmpPaths,previousPaths);
               end       

               % remove all loop paths
               tmpPaths(tmpPaths==-1,:) = [];
               paths = Util.appendMatrix(paths,tmpPaths);
           end
        end
        
        function type = getBlockType(path)
            if iscell(path)
                s = size(path);
                type = cell(s);
                for i = 1:s
                    type{i} = getBlockType(path{i});
                end
            elseif size(path,1) > 1
                s = size(path,1);
                type = cell(s);
                for i = 1:s
                    type{i} = getBlockType(path(i,1));
                end
            else        
                if strcmp(Util.getBlockName(path), 'True') || strcmp(Util.getBlockName(path), 'False')
                    type = 'Constant';
                elseif Util.hasParam(path, 'BlockType')
                    type = get_param(path, 'BlockType');
                elseif strcmp(get_param(path, 'Type'), 'block_diagram')
                    type = 'SubSystem';
                end
                
                if strcmp(Util.getBlockName(path), 'VERSION_INFO')
                    type = 'VERSION_INFO';
                elseif strcmp(Util.getBlockName(path), 'SysInit')
                    type = 'SysInit';
                end
            end
        end
       
        function name = getBlockName(path)
            if iscell(path)
                s = size(path);
                name = cell(s);
                for i = 1:s
                    name{i} = Util.getBlockName(path{i});
                end
            else
                if Util.hasParam(path, 'Name')
                    name = get_param(path, 'Name');
                elseif strcmp(get_param(path, 'Name'), 'block_diagram')
                    name = 'SubSystem';
                end
            end
            
             % remove whitespaces from names
            name = strrep(name, ' ', '');
        end
        
        function numInputs = getNumberOfInputs(path)
            pCon = get_param(path, 'PortConnectivity');
            inputs = {pCon.SrcBlock};
            inputs = inputs(~cellfun('isempty',inputs));
            
            numInputs = size(inputs, 2);
        end
        
        function b = hasParam(blockPath, param)
            b = isfield(get_param(blockPath, 'ObjectParameters'), param);
        end

        function handles = convertFASDiaryEntries(diaryPath)
            file = fopen(diaryPath);
            out = [];

            while ~feof(file)
                line = fgets(file);
                tmp = strsplit(line, '''');

                if(numel(tmp) > 1)
                    path = tmp{2};

                    if ~isempty(strfind(path,'Oeffentlicher'))
                        try
                            handle = get_param(path, 'Handle');
                            out = [out; handle];
                        catch
                        end
                    end
                end        
            end

            fclose(file);
            handles = out;
        end

        function m = appendMatrix(m1,m2)
            s11 = size(m1,1);
            s21 = size(m2,1);
            s22 = size(m2,2);

            m1(s11+1:(s11+s21),1:s22) = m2;

            m = m1;
        end

    end    
end

