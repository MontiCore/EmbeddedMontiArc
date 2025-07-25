/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.util.types.SimLangEnums;

public class SimulationTypeSymbol extends CommonSymbol{

  public static final SimulationTypeSymbolKind KIND = SimulationTypeSymbolKind.INSTANCE;
  private SimLangEnums.SimulationTypes simulationType;

  public SimulationTypeSymbol(String name, SimLangEnums.SimulationTypes simulationType) {
    super(name, KIND);
    this.simulationType = simulationType;
  }

  public SimLangEnums.SimulationTypes getSimulationType() {
    return simulationType;
  }
}
