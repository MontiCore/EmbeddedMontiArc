// (c) https://github.com/MontiCore/monticore 
classdef CompPower < AbstractNFP
    properties
    end
    
    methods
        function obj = CompPower(valueFile)
            aggOp = @(x,y)(x+y);
            obj@AbstractNFP(valueFile, aggOp);
            obj.name = 'CompPower';  
            obj.unit = 'nJ';
        end
    end    
end

