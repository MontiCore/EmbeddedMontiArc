// (c) https://github.com/MontiCore/monticore 
function type = getBlockType( path )
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
        if strcmp(getBlockName(path), 'True') || strcmp(getBlockName(path), 'False')
            type = 'Constant';
        elseif hasParam(path, 'BlockType')
            type = get_param(path, 'BlockType');
        elseif strcmp(get_param(path, 'Type'), 'block_diagram')
            type = 'SubSystem';
        end
    end
end

