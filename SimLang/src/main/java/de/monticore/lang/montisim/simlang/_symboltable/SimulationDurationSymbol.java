package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.simlang.util.ValueListRangeLambda;

public class SimulationDurationSymbol extends CommonSymbol{

  public static final SimulationDurationSymbolKind KIND = SimulationDurationSymbolKind.INSTANCE;
  private ValueListRangeLambda simulationDuration;

  public SimulationDurationSymbol(String name, ValueListRangeLambda simulationDuration) {
    super(name, KIND);
    this.simulationDuration = simulationDuration;
  }

  public ValueListRangeLambda getSimulationRenderFrequency() {
    return simulationDuration;
  }
}
