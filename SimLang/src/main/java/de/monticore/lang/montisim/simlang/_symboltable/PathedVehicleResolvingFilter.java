/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class PathedVehicleResolvingFilter extends CommonResolvingFilter<PathedVehicleSymbol> {
  public PathedVehicleResolvingFilter() {
    super(PathedVehicleSymbol.KIND);
  }
}
