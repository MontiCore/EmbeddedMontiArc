/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class SimulationRenderFrequencyResolvingFilter extends CommonResolvingFilter<SimulationRenderFrequencySymbol> {
  public SimulationRenderFrequencyResolvingFilter() {
    super(SimulationRenderFrequencySymbol.KIND);
  }
}
