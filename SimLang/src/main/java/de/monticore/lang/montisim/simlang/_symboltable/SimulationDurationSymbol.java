/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class SimulationDurationSymbol extends CommonSymbol{

  public static final SimulationDurationKind KIND = SimulationDurationKind.INSTANCE;
  private AlternativeInput simulationDuration;

  public SimulationDurationSymbol(String name, AlternativeInput simulationDuration) {
    super(name, KIND);
    this.simulationDuration = simulationDuration;
  }

  public AlternativeInput getSimulationDuration() {
    return simulationDuration;
  }
}
