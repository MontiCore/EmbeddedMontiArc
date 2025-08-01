// (c) https://github.com/MontiCore/monticore 
function getComponentOutputsTest
    display('running getComponentOutputsTest...');
    
    testCCOOOutputs;
    testER1Outputs;
    
    display('getComponentOutputsTest successful.');
end

function testCCOOOutputs
   errMsg = 'Test "getComponentOutputsTest" failed for Component "CC_On_Off".';
   ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
   
   output = getComponentOutputs(ccoo);
   assert(~isempty(output), errMsg);
   
   name = get_param(output,'Name');
   expected = 'CC_active_b';
   
   assert(strcmp(name,expected), errMsg);
end

function testER1Outputs
   errMsg = 'Test "getComponentOutputsTest" failed for Component "EdgeRising1".';
   ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
   er1 = [ccoo '/EdgeRising1'];
   
   output = getComponentOutputs(er1);
   assert(~isempty(output), errMsg);
   
   name = get_param(output,'Name');
   expected = 'y';
   
   assert(strcmp(name,expected), errMsg);
end
