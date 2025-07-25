// (c) https://github.com/MontiCore/monticore 
function computeBCETTest
    display('running computeBCETTest ...');
    
    testEdgeRising;
    testCCOnOff;
    
    display('computeBCETTest successful.');
end

function testEdgeRising
    ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
    component = [ccoo '/EdgeRising1'];
    bcet = getBCETForComponent(component);
    expected = 1;
    
    assert(bcet == expected, 'Testing BCET for EdgeRising failed.')
end

function testCCOnOff
    ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
    bcet = getBCETForComponent(ccoo);
    expected = 3;
    
    assert(bcet == expected, 'Testing BCET for CC_On_Off failed.')
end
