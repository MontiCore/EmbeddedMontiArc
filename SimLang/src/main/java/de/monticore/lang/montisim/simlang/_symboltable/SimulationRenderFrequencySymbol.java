package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.simlang.util.ValueListRangeLambda;

public class SimulationRenderFrequencySymbol extends CommonSymbol{

  public static final SimulationRenderFrequencySymbolKind KIND = SimulationRenderFrequencySymbolKind.INSTANCE;
  private ValueListRangeLambda simulationRenderFrequency;

  public SimulationRenderFrequencySymbol(String name, ValueListRangeLambda simulationRenderFrequency) {
    super(name, KIND);
    this.simulationRenderFrequency = simulationRenderFrequency;
  }

  public ValueListRangeLambda getSimulationRenderFrequency() {
    return simulationRenderFrequency;
  }
}
