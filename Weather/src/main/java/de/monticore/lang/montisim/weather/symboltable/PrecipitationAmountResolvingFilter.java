/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class PrecipitationAmountResolvingFilter extends CommonResolvingFilter<PrecipitationAmountSymbol> {
  public PrecipitationAmountResolvingFilter() {
    super(PrecipitationAmountSymbol.KIND);
  }
}
