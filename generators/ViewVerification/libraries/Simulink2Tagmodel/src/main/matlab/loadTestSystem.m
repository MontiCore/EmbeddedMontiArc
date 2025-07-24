// (c) https://github.com/MontiCore/monticore 
function [ccoo,er1,rsff] = loadTestSystem
    % for testing
    load_system('Oeffentlicher_Demonstrator_FAS_v04');
    
    ccoo = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off';
    er1 = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off/EdgeRising1';
    rsff = 'Oeffentlicher_Demonstrator_FAS_v04/DEMO_FAS/DEMO_FAS/Subsystem/DEMO_FAS/DEMO_FAS_Funktion/Tempomat/Tempomat_Function/CC_On_Off/RSFlipFlop';
end

