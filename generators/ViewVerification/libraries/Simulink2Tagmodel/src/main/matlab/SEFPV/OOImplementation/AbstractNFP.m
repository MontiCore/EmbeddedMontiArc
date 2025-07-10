// (c) https://github.com/MontiCore/monticore 
classdef (Abstract = true) AbstractNFP < handle
    %AbstractNFP Stores and provides concrete NFP values 
    %   Subclasses define a cell array of Strings and Integers,
    %   which is used as a map to store and retrieve NFP values
    
    properties (SetAccess = public, GetAccess = public)
        name
        values
        unit
        aggregationOperator
    end
    
    methods (Access = public)
        function obj = AbstractNFP(valueFile, aggOp)
           obj.values = obj.readValuesFromFile(valueFile); 
           obj.aggregationOperator = aggOp;
        end
        
        function value = getValueForKey(obj, key) 
            map = strcmp(key,obj.values);
            
            if sum(map(:,1) > 0)
                value = obj.values{find(map),2};
            else
                % key not found, output a warning and return 0
                fprintf('WARNING: %s of BlockType \"%s\" is unknown (will be ignored in the calculation).\n', obj.name, key);
                value = 0;
            end
        end
        
        function value = getValueForBlock(obj, block)
           type = Util.getBlockType(block);
           addTime = 0;
           
           switch type
               case 'Logic'
                   op = get_param(block, 'operator');
                   numInputs = Util.getNumberOfInputs(block);
               
                   type = strcat(op, num2str(numInputs));
               case 'Product'
                   numInputs = Util.getNumberOfInputs(block);
                   
                   type = strcat('Product', num2str(numInputs));
               case 'Math'
                   op = get_param(block, 'operator');
                   numInputs = Util.getNumberOfInputs(block);
                   
                   type = strcat(type, '_', op, num2str(numInputs));                   
               case 'If'
                   expr = get_param(block, 'IfExpression');
                   if Util.hasParam(block, 'ElseIfExpressions')
                      elif = get_param(block, 'ElseIfExpressions');
                   end
                   
                   exp = expr;
                   num = size(strfind(exp, '&'), 2);
                   addTime = num * obj.getValueForKey('AND2');
                   num = size(strfind(exp, '|'), 2);
                   addTime = addTime + num * obj.getValueForKey('OR2');
                   num = size(strfind(exp, '<'), 2);
                   addTime = addTime + num * obj.getValueForKey('RelationalOperator');
                   num = size(strfind(exp, '>'), 2);
                   addTime = addTime + num * obj.getValueForKey('RelationalOperator');
                   
                   if(size(elif,1) > 1)
                       elif = elif{1};
                       warning('Check ElIf-Expression in AbstractNFP.');
                   end
                   exp = elif;
                   num = size(strfind(exp, '&'), 2);
                   addTime = num * obj.getValueForKey('AND2');
                   num = size(strfind(exp, '|'), 2);
                   addTime = addTime + num * obj.getValueForKey('OR2');
                   num = size(strfind(exp, '<'), 2);
                   addTime = addTime + num * obj.getValueForKey('RelationalOperator');
                   num = size(strfind(exp, '>'), 2);
                   addTime = addTime + num * obj.getValueForKey('RelationalOperator');
               case {'Switch', 'MultiPortSwitch'}
                   numInputs = Util.getNumberOfInputs(block);
                   num = numInputs - 1;
                   
                   type = strcat('Switch', num2str(num));
               otherwise
                   % nothing
           end
           
           tmp = obj.getValueForKey(type);
           value = tmp + addTime;
        end
    end
    methods (Static)
        function values = readValuesFromFile(file)
            try
              f = fopen(file);
              
              values = {};
              while ~feof(f)
                 line = fgets(f);
                 parts = strsplit(line, ' ');
                                  
                 values(size(values,1)+1,1) = parts(1);
                 values{size(values,1),  2} = str2double(strrep(parts(2), ',', '.'));
              end
            catch
              error('Cannot open values file. Please check the file name.');
            end
        end
    end    
end

