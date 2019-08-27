/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class SimulationTypeResolvingFilter extends CommonResolvingFilter<SimulationTypeSymbol> {
  public SimulationTypeResolvingFilter() {
    super(SimulationTypeSymbol.KIND);
  }
}
