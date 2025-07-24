/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class SimulationLoopFrequencyResolvingFilter extends CommonResolvingFilter<SimulationLoopFrequencySymbol> {
  public SimulationLoopFrequencyResolvingFilter() {
    super(SimulationLoopFrequencySymbol.KIND);
  }
}
