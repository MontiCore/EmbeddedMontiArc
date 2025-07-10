// (c) https://github.com/MontiCore/monticore 
function stereotype = getBlockStereotype( blockPath )
%GETBLOCKSTEREOTYPE Summary of this function goes here
%   Detailed explanation goes here
    type = getBlockType(blockPath);
    
    if strcmp(type, 'SubSystem') && hasParam(blockPath, 'Ports')
        if isEnabledSubsystem(blockPath)
            stereotype = addStereotype('', 'Type', 'EnabledSubSystem');
        elseif isIfActionSubsystem(blockPath)
            stereotype = addStereotype('', 'Type', 'IfActionSubSystem');
        elseif isTriggeredSubsystem(blockPath)
            stereotype = addStereotype('', 'Type', 'TriggeredSubSystem');
        else
            stereotype = addStereotype('', 'Type', type);
        end
    else
        stereotype = addStereotype('', 'Type', type);
    end
    
    if strcmp(type, 'If')
        stereotype = addStereotype(stereotype, 'Expression', getIfExpressions(blockPath));
        stereotype = addStereotype(stereotype, 'ShowElse', get_param(blockPath, 'ShowElse'));
        
        
    elseif strcmp(type, 'Constant')
        stereotype = addStereotype(stereotype, 'Value', getConstantValue(blockPath));
        
    elseif strcmp(type, 'Bias')
        stereotype = addStereotype(stereotype, 'Bias', get_param(blockPath, 'Bias'));

    elseif strcmp(type, 'Gain')
        stereotype = addStereotype(stereotype, 'Gain', get_param(blockPath, 'Gain'));

    elseif strcmp(type,  'DataStoreMemory') || strcmp(type,  'DataStoreRead') || strcmp(type,  'DataStoreWrite')
        stereotype = addStereotype(stereotype, 'Variable', get_param(blockPath, 'DataStoreName'));
        if strcmp(type,  'DataStoreMemory')
            stereotype = addStereotype(stereotype, 'InitialValue', get_param(blockPath, 'InitialValue'));
        end
        
    elseif strcmp(type, 'BusSelector')
        stereotype = addStereotype(stereotype, 'OutputAsBus', get_param(blockPath, 'OutputAsBus'));
    
    elseif strcmp(type, 'TriggerPort')
        stereotype = addStereotype(stereotype, 'TriggerType', get_param(blockPath, 'TriggerType'));
    
    elseif strcmp(type, 'Switch')
        stereotype = addStereotype(stereotype, 'Criteria', get_param(blockPath, 'Criteria'));
        stereotype = addStereotype(stereotype, 'Threshold', get_param(blockPath, 'Threshold'));
        
    elseif strcmp(type, 'Merge')
        stereotype = addStereotype(stereotype, 'InitialOutput', get_param(blockPath, 'InitialOutput'));
        
    elseif strcmp(type, 'Logic') || strcmp(type, 'RelationalOperator')
        stereotype = addStereotype(stereotype, 'Operator', get_param(blockPath, 'Operator'));
        
    elseif strcmp(type, 'MinMax')
        stereotype = addStereotype(stereotype, 'Function', get_param(blockPath, 'Function'));
        
    elseif strcmp(type, 'Math')
        stereotype = addStereotype(stereotype, 'Function', get_param(blockPath, 'Function'));
        
    elseif strcmp(type, 'Sum')
        stereotype = addStereotype(stereotype, 'ListOfSigns', getSumSigns(blockPath));
        
    elseif strcmp(type, 'Product')
        stereotype = addStereotype(stereotype, 'Inputs', get_param(blockPath, 'Inputs'));
    
    elseif strcmp(type, 'Saturate')
        stereotype = addStereotype(stereotype, 'LowerLimit', get_param(blockPath, 'LowerLimit'));
        stereotype = addStereotype(stereotype, 'UpperLimit', get_param(blockPath, 'UpperLimit'));
        
    elseif strcmp(type, 'Lookup_n-D')
        [keys, values] = getLookupTableStereotypes(blockPath);
        for i = 1:numel(keys)
            stereotype = addStereotype(stereotype, keys{i}, values{i});
        end
        
    elseif strcmp(type, 'MultiPortSwitch')
        stereotype = addStereotype(stereotype, 'DataPortOrder', get_param(blockPath, 'DataPortOrder'));
        
    elseif strcmp(type, 'UnitDelay')
        stereotype = addStereotype(stereotype, 'InitialCondition', get_param(blockPath, 'InitialCondition'));
    end

    stereotype = createStereotype(stereotype);
end

function value = getConstantValue(constantPath)
    value = get_param(constantPath, 'Value');
    
    %check if constant is a variable in workspace
    if evalin('base', ['exist(''' value ''', ''var'')']);
        value = evalin('base', ['eval(''' value ''')']);
    end
end

function signs = getSumSigns(sumPath)
    signs = get_param(sumPath, 'ListOfSigns');
    signs = strrep(signs, '|', '');
end

function expression = getIfExpressions(blockPath)
    ifexpression = get_param(blockPath, 'IfExpression');
    elseifexpressions = get_param(blockPath, 'ElseIfExpressions');
    
    ifexpression = strrep(ifexpression, 'u', 'condition');
    elseifexpressions = strrep(elseifexpressions, 'u', 'condition');
    
    if isempty(elseifexpressions)
        expression = ifexpression;
    else
        expression = [ifexpression ',' elseifexpressions];
    end
end

function [keys, values] = getLookupTableStereotypes(lookupPath)
    keys = cell(3,1);
    values = cell(3,1);
    keys{1} = 'NumberOfTableDimensions';
    keys{2} = 'Table';
    keys{3} = 'BreakpointsSpecification';
    values{1} = get_param(lookupPath, keys{1});
    values{2} = get_param(lookupPath, keys{2});
    values{3} = get_param(lookupPath, keys{3});
    
    dim = str2num(values{1});
    for i = 1:dim
       index = 3+i;
       keys{index} = ['BreakpointsForDimension' num2str(i)];
       values{index} = get_param(lookupPath, keys{index});
    end
end
