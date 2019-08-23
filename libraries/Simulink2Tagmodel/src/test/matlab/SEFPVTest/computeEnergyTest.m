// (c) https://github.com/MontiCore/monticore 
function computeWCETTest
    display('running computeEnergyTest ...');
    
    testEdgeRising_without_eo;
    testEdgeRising_with_eo;
    testCCOnOff_without_eo;
    testCCOnOff_with_eo;
    
    display('computeEnergyTest successful.');
end

function testEdgeRising_without_eo
    ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
    component = [ccoo '/EdgeRising1'];
    energy = getEnergyForComponent(component);
    expected = 5;
    
    assert(energy == expected, 'Testing getEnergy for EdgeRising without EO failed.')
end

function testEdgeRising_with_eo
    ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
    component = [ccoo '/EdgeRising1'];
    energy = getEnergyForComponent(component, 'diary_FAS.txt');
    expected = 60;
    
    assert(energy == expected, 'Testing getEnergy for EdgeRising with EO failed.')
end

function testCCOnOff_without_eo
    ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
    energy = getEnergyForComponent(ccoo);
    expected = 15;
    
    assert(energy == expected, 'Testing getEnergy for CC_On_Off without EO failed.')
end

function testCCOnOff_with_eo
    ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
    energy = getEnergyForComponent(ccoo, 'diary_FAS.txt');
    expected = 72;
    
    assert(energy == expected, 'Testing getEnergy for CC_On_Off with EO failed.')
end
