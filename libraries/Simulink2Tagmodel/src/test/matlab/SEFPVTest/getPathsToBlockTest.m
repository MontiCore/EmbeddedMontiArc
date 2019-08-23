// (c) https://github.com/MontiCore/monticore 
function getPathsToBlockTest
    display('running getPathsToBlockTest...');
    
    testCCOOPaths;
    testER1Paths;
    
    display('getPathsToBlockTest successful.');
end

function testCCOOPaths
   errMsg = 'Test "getPathsToBlockTest" failed for Component "CC_On_Off".';
   ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
   
   output = getComponentOutputs(ccoo);
   paths = getPathsToBlock(output);
   assert(~isempty(paths), errMsg);
   
   % check for known path
   paths = getBlockNames(paths);
   assert(strcmp(paths{15,1}, 'ParkingBrake_b'), errMsg);
   assert(strcmp(paths{15,2}, 'Logical Operator'), errMsg);
   assert(strcmp(paths{15,3}, 'RSFlipFlop'), errMsg);
   assert(strcmp(paths{15,4}, 'CC_active_b'), errMsg);
   assert(strcmp(paths{15,5}, ''), errMsg);
end

function testER1Paths
   errMsg = 'Test "getPathsToBlockTest" failed for Component "EdgeRising1".';
   ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
   er1 = [ccoo '/EdgeRising1'];
   
   output = getComponentOutputs(er1);
   paths = getPathsToBlock(output);
   assert(~isempty(paths), errMsg);
   
   % check for known path
   paths = getBlockNames(paths);
   assert(strcmp(paths{1,1}, 'u'), errMsg);
   assert(strcmp(paths{1,2}, 'LogOp_A'), errMsg);
   assert(strcmp(paths{1,3}, 'y'), errMsg);
end
