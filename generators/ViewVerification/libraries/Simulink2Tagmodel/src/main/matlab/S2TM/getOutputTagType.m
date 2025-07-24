// (c) https://github.com/MontiCore/monticore 
function tagTypeString = getOutputTagType(tagType)
    switch(tagType)
        case {'WCET','BCET'}
            tagTypeString = 'LatencyCmpInst';
        case 'CompPower'
            tagTypeString = 'CompPowerInst';
        otherwise
            error('Unknown tag type.');
    end
end

