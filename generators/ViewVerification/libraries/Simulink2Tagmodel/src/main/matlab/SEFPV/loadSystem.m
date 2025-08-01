// (c) https://github.com/MontiCore/monticore 
function [] = loadSystem(expanded)
    if expanded
        load_system('Oeffentlicher_Demonstrator_FAS_v04_expanded')
    else
        load_system('Oeffentlicher_Demonstrator_FAS_v04')
    end
end

