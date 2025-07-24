// (c) https://github.com/MontiCore/monticore 
function [paths, blocksVisited] = getPathsToBlock(path, blocksVisited)
   if nargin < 2
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
               previousPaths = getPathsToBlock(previousBlock, blocksVisited);
               
           end
           
           % append current block to previous paths
           for j = 1:size(previousPaths,1)
               previousPaths(j,find(previousPaths(j,:),1,'last')+1) = get_param(path, 'Handle');
           end
                     
           tmpPaths = appendMatrix(tmpPaths,previousPaths);
       end       
       
       % remove all loop paths
       tmpPaths(tmpPaths==-1,:) = [];
       paths = appendMatrix(paths,tmpPaths);
   end
end
