function isSys = isSubsystem(blockPath)
%ISSUBSYSTEM Summary of this function goes here
%   Detailed explanation goes here
    isSys = strcmp(getBlockType(blockPath), 'SubSystem');
end

