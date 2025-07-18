/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class SimulationRenderFrequencySymbol extends CommonSymbol{

  public static final SimulationRenderFrequencyKind KIND = SimulationRenderFrequencyKind.INSTANCE;
  private AlternativeInput simulationRenderFrequency;

  public SimulationRenderFrequencySymbol(String name, AlternativeInput simulationRenderFrequency) {
    super(name, KIND);
    this.simulationRenderFrequency = simulationRenderFrequency;
  }

  public AlternativeInput getSimulationRenderFrequency() {
    return simulationRenderFrequency;
  }
}
