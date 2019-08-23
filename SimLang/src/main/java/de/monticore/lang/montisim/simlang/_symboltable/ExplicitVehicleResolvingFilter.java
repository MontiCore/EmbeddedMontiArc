/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class ExplicitVehicleResolvingFilter extends CommonResolvingFilter<ExplicitVehicleSymbol> {
  public ExplicitVehicleResolvingFilter() {
    super(ExplicitVehicleSymbol.KIND);
  }
}
