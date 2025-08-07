// (c) https://github.com/MontiCore/monticore 
function computeWCETTest
    display('running computeWCETTest ...');
    
    testEdgeRising_without_eo;
    testEdgeRising_with_eo;
    testCCOnOff_without_eo;
    testCCOnOff_with_eo;
    
    display('computeWCETTest successful.');
end

function testEdgeRising_without_eo
    ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
    component = [ccoo '/EdgeRising1'];
    wcet = getWCETForComponent(component);
    expected = 5;
    
    assert(wcet == expected, 'Testing WCET for EdgeRising without EO failed.')
end

function testEdgeRising_with_eo
    ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
    component = [ccoo '/EdgeRising1'];
    wcet = getWCETForComponent(component, 'diary_FAS.txt');
    expected = 60;
    
    assert(wcet == expected, 'Testing WCET for EdgeRising with EO failed.')
end

function testCCOnOff_without_eo
    ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
    wcet = getWCETForComponent(ccoo);
    expected = 15;
    
    assert(wcet == expected, 'Testing WCET for CC_On_Off without EO failed.')
end

function testCCOnOff_with_eo
    ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
    wcet = getWCETForComponent(ccoo, 'diary_FAS.txt');
    expected = 72;
    
    assert(wcet == expected, 'Testing WCET for CC_On_Off with EO failed.')
end
