package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.simlang.util.ValueListRangeLambda;

public class SimulationLoopFrequencySymbol extends CommonSymbol{

  public static final SimulationLoopFrequencySymbolKind KIND = SimulationLoopFrequencySymbolKind.INSTANCE;
  private ValueListRangeLambda simulationLoopFrequency;

  public SimulationLoopFrequencySymbol(String name, ValueListRangeLambda simulationLoopFrequency) {
    super(name, KIND);
    this.simulationLoopFrequency = simulationLoopFrequency;
  }

  public ValueListRangeLambda getSimulationLoopFrequency() {
    return simulationLoopFrequency;
  }
}
