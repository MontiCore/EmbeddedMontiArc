// (c) https://github.com/MontiCore/monticore 
classdef WCET < AbstractNFP
    properties
    end
    
    methods
        function obj = WCET(valueFile)
            aggOp = @(x,y)(x+y);
            obj@AbstractNFP(valueFile, aggOp);
            obj.name = 'WCET';   
            obj.unit = 'ns';
        end
    end    
end

