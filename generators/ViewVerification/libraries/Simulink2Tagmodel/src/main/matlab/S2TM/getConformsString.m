// (c) https://github.com/MontiCore/monticore 
function conformsString = getConformsString(tagType)
    pref = 'conforms to nfp.';
    suff = ';\n';

    tagSchema = '';
    switch(tagType)
        case 'Latency'
            tagSchema = 'TagLatencyTagSchema';
        case 'CompPower'
            tagSchema = 'CompPower';
        otherwise
            error('Unknown tag type.');
    end
    
    conformsString = strcat(pref,tagSchema,suff);
end

