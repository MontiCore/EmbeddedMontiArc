package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.simlang.util.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class SimulationLoopFrequencySymbol extends CommonSymbol{

  public static final SimulationLoopFrequencySymbolKind KIND = SimulationLoopFrequencySymbolKind.INSTANCE;
  private AlternativeInput simulationLoopFrequency;

  public SimulationLoopFrequencySymbol(String name, AlternativeInput simulationLoopFrequency) {
    super(name, KIND);
    this.simulationLoopFrequency = simulationLoopFrequency;
  }

  public AlternativeInput getSimulationLoopFrequency() {
    return simulationLoopFrequency;
  }
}
