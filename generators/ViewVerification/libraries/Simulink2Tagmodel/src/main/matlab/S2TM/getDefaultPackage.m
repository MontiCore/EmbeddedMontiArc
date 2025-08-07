// (c) https://github.com/MontiCore/monticore 
function package = getDefaultPackage(path)
%GETDEFAULTPACKAGE Summary of this function goes here
%   Detailed explanation goes here
    if ~exist('path', 'var')
        path = getDefaultPath;
    end
    
    [~, folder] = fileparts(path);
    package = changeFirstCharacterCase(folder, true);
end

