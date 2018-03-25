package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.simlang.util.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class SimulationDurationSymbol extends CommonSymbol{

  public static final SimulationDurationSymbolKind KIND = SimulationDurationSymbolKind.INSTANCE;
  private AlternativeInput simulationDuration;

  public SimulationDurationSymbol(String name, AlternativeInput simulationDuration) {
    super(name, KIND);
    this.simulationDuration = simulationDuration;
  }

  public AlternativeInput getSimulationDuration() {
    return simulationDuration;
  }
}
