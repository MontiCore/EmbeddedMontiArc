/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class SimulationLoopFrequencySymbol extends CommonSymbol{

  public static final SimulationLoopFrequencyKind KIND = SimulationLoopFrequencyKind.INSTANCE;
  private AlternativeInput simulationLoopFrequency;

  public SimulationLoopFrequencySymbol(String name, AlternativeInput simulationLoopFrequency) {
    super(name, KIND);
    this.simulationLoopFrequency = simulationLoopFrequency;
  }

  public AlternativeInput getSimulationLoopFrequency() {
    return simulationLoopFrequency;
  }
}
