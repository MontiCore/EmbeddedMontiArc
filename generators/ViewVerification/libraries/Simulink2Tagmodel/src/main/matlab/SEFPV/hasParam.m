// (c) https://github.com/MontiCore/monticore 
function b = hasParam( blockPath, param )
%HASPARAM Summary of this function goes here
%   Detailed explanation goes here
    b = isfield(get_param(blockPath, 'ObjectParameters'), param);
end

