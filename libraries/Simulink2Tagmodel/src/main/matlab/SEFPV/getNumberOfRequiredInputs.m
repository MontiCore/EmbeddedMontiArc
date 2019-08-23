// (c) https://github.com/MontiCore/monticore 
function num = getNumberOfRequiredInputs(blockPath)
    inputs = getInputHandles(blockPath);
    
    numInputs = size(inputs,2);
    
    if isMask(blockPath) & numInputs > 0       
        type = get_param(blockPath, 'MaskType');
        switch type
            case 'TLMSR_V200_EdgeRising'
                num = [1 numInputs];
            case 'TLMSR_V200_RSFlipFlop'
                num = [2 numInputs];
            case 'TL_LogicalOperator'
                num = [1 numInputs];
            case 'TL_Switch'
                num = [1 numInputs];
            case 'TL_Sum'
                num = [2 2];
            case 'TL_RelationalOperator'
                num = [2 2];
            case 'TL_Gain'
                num = [1 1];
            otherwise
                fprintf('WARNING: Number of required inputs for mask type \"%s\" unknown.\n', type);
                num = [numInputs numInputs];
        end 
    else
        num = [numInputs numInputs];
    end
end

